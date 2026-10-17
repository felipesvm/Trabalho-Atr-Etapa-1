#include "navegacao.hpp"

#include <algorithm>
#include <cstdio>

#include "../comum/console.hpp"

namespace amr {

namespace {
constexpr double MARGEM_PASSAGEM = 0.3;
constexpr int64_t PRAZO_ROTA_NS = 1 * NS_POR_S;

bool estado_de_equipe(EstadoRobo e) {
    return e == EstadoRobo::AGUARDANDO_EQUIPE || e == EstadoRobo::ACOPLANDO || e == EstadoRobo::TRANSPORTANDO ||
           e == EstadoRobo::DESACOPLANDO || e == EstadoRobo::ABORTANDO;
}

void abortar_equipe(SlotEquipe& eq, uint32_t epoca) {
    TravaCompartilhada t(&eq.mtx);
    if (eq.epoca == epoca && eq.estado != EstadoEquipe::LIVRE && eq.estado != EstadoEquipe::CONCLUIDA)
        eq.estado = EstadoEquipe::ABORTANDO;
}
}  // namespace

Navegacao::Navegacao(NucleoRobo& r) : r_(r), emissor_(r.org) {}

Navegacao::VisaoContexto Navegacao::ler_contexto() {
    std::lock_guard<std::mutex> t(r_.mtx_nav);
    const auto& c = r_.ctx;
    return {c.estado,         c.modo,      c.inibido,  c.parado, c.sem_comunicacao, c.bateria_critica,
            c.recarga_pendente, c.encerrar, c.missao};
}

bool Navegacao::mudar_estado(EstadoRobo de, EstadoRobo para) {
    {
        std::lock_guard<std::mutex> t(r_.mtx_nav);
        if (r_.ctx.estado != de) return false;
        r_.ctx.estado = para;
    }
    LOG("T_NAV", "estado %s -> %s", nome_estado(de), nome_estado(para));
    return true;
}

void Navegacao::processar_comandos(const VisaoContexto& c, const Ponto& pos) {
    Mensagem m;
    unsigned prio = 0;
    while (r_.comandos.receber(&m, sizeof m, 0, &prio)) {
        LOG("T_NAV", "comando %s seq=%u prio=%u", nome_tipo(m.cab.tipo), m.cab.seq, prio);
        EstadoRobo estado;
        {
            std::lock_guard<std::mutex> t(r_.mtx_nav);
            estado = r_.ctx.estado;
        }
        switch (m.cab.tipo) {
            case TipoMsg::PARAR: {
                std::lock_guard<std::mutex> t(r_.mtx_nav);
                r_.ctx.parado = true;
                break;
            }
            case TipoMsg::RETOMAR: {
                std::lock_guard<std::mutex> t(r_.mtx_nav);
                r_.ctx.parado = false;
                break;
            }
            case TipoMsg::MODO: {
                std::lock_guard<std::mutex> t(r_.mtx_nav);
                r_.ctx.modo = m.comando.modo;
                break;
            }
            case TipoMsg::DESTINO_MANUAL:
                if (estado == EstadoRobo::DISPONIVEL || (estado == EstadoRobo::DESLOCANDO && objetivo_ == Objetivo::MANUAL)) {
                    objetivo_ = Objetivo::MANUAL;
                    destino_ = {m.comando.x, m.comando.y};
                    pedir_rota(pos, destino_, {}, Finalidade::PROPRIA);
                    if (estado == EstadoRobo::DISPONIVEL) mudar_estado(estado, EstadoRobo::DESLOCANDO);
                } else {
                    LOG("T_NAV", "destino manual recusado no estado %s", nome_estado(estado));
                }
                break;
            case TipoMsg::MISSAO_EQUIPE: {
                Missao nova;
                nova.ativa = true;
                nova.equipe = m.missao.equipe;
                nova.epoca = m.missao.epoca;
                nova.papel = m.missao.papel;
                nova.indice = m.missao.indice;
                nova.caixa = m.missao.caixa;
                nova.k = m.missao.k;
                nova.afastamento = {m.missao.off_x, m.missao.off_y};
                nova.pr = {m.missao.pr_x, m.missao.pr_y};
                nova.zona = {m.missao.zona_x, m.missao.zona_y};
                bool equipe_valida = nova.equipe >= 0 && uint32_t(nova.equipe) < r_.frota->n_max() && nova.k >= 1 &&
                                     nova.k <= MAX_MEMBROS && nova.indice < nova.k;
                if (equipe_valida && estado == EstadoRobo::DISPONIVEL && !c.recarga_pendente && !c.missao.ativa)
                    iniciar_missao(nova, pos);
                else
                    LOG("T_NAV", "missão da caixa %d recusada no estado %s", nova.caixa, nome_estado(estado));
                break;
            }
            case TipoMsg::RECONHECER_FALHA: {
                bool liberado = false;
                bool falha = r_.imagem.ler_entradas().falha;
                {
                    std::lock_guard<std::mutex> t(r_.mtx_nav);
                    if (r_.ctx.estado == EstadoRobo::DEFEITO && !falha) {
                        r_.ctx.inibido = false;
                        r_.ctx.estado = EstadoRobo::DISPONIVEL;
                        liberado = true;
                    }
                }
                LOG("T_NAV", liberado ? "falha reconhecida, robô liberado" : "reconhecimento ignorado: defeito ainda presente");
                break;
            }
            case TipoMsg::ABORTAR:
                if (c.missao.ativa) abortar_equipe(r_.frota->equipe(uint32_t(c.missao.equipe)), c.missao.epoca);
                break;
            case TipoMsg::ENCERRAR: {
                std::lock_guard<std::mutex> t(r_.mtx_nav);
                r_.ctx.encerrar = true;
                r_.ativo = false;
                break;
            }
            default: break;
        }
    }
}

void Navegacao::avaliar_prazo(const InfoCiclo& info) {
    if (info.perdas_consecutivas >= 3 && !congelado_prazo_) {
        congelado_prazo_ = true;
        Mensagem m = r_.evento(TipoMsg::DEADLINE_NAV);
        m.evento.extra = int32_t(info.perdas_consecutivas);
        emissor_.enviar(m);
    } else if (info.perdas_consecutivas == 0) {
        congelado_prazo_ = false;
    }
}

// ------------------------------------------------------------------ rotas

void Navegacao::pedir_rota(const Ponto& origem, const Ponto& destino, OpcoesRota op, Finalidade f) {
    for (int p : passagens_evitadas_) op.bloqueadas.push_back(p);
    {
        std::lock_guard<std::mutex> t(r_.mtx_nav);
        auto& ped = r_.ctx.pedido;
        ped.versao++;
        ped.origem = origem;
        ped.destino = destino;
        ped.opcoes = op;
        ped.finalidade = f;
    }
    r_.cv_rota.notify_one();
    seguidor_.limpar();
    finalidade_rota_ = f;
    t_pedido_ = agora_ns();
    timeout_rota_avisado_ = false;
    aguardando_rota_ = true;
}

bool Navegacao::rota_nova(Rota& rota, bool& ok) {
    std::lock_guard<std::mutex> t(r_.mtx_nav);
    auto& pl = r_.ctx.planejada;
    if (pl.versao != r_.ctx.pedido.versao || pl.versao == versao_aplicada_) return false;
    versao_aplicada_ = pl.versao;
    ok = pl.ok;
    rota = pl.rota;
    return true;
}

bool Navegacao::verificar_rota(const Ponto& pos, Rota& rota, bool& ok) {
    if (rota_nova(rota, ok)) {
        aguardando_rota_ = false;
        return true;
    }
    if (!timeout_rota_avisado_ && agora_ns() - t_pedido_ > PRAZO_ROTA_NS) {
        timeout_rota_avisado_ = true;
        emissor_.enviar(r_.evento(TipoMsg::ROTA_TIMEOUT));
        PedidoRota anterior;
        {
            std::lock_guard<std::mutex> t(r_.mtx_nav);
            anterior = r_.ctx.pedido;
        }
        OpcoesRota simplificada = anterior.opcoes;
        simplificada.somente_corredores = true;
        pedir_rota(anterior.finalidade == Finalidade::CARGA ? anterior.origem : pos, anterior.destino, simplificada,
                   anterior.finalidade);
        timeout_rota_avisado_ = true;
    }
    return false;
}

Navegacao::Andamento Navegacao::seguir(const Ponto& pos, Saidas& s, bool segurar, EstadoRobo estado) {
    if (aguardando_rota_) {
        Rota rota;
        bool ok = false;
        if (!verificar_rota(pos, rota, ok)) {
            s.sp_x = pos.x;
            s.sp_y = pos.y;
            return Andamento::EM_CURSO;
        }
        if (!ok) {
            Mensagem m = r_.evento(TipoMsg::SEM_ROTA);
            m.evento.x = float(destino_.x);
            m.evento.y = float(destino_.y);
            emissor_.enviar(m);
            return Andamento::FALHOU;
        }
        seguidor_.definir(std::move(rota));
    }
    if (seguidor_.concluida()) return Andamento::CHEGOU;
    const Waypoint* alvo = seguidor_.alvo();
    s.sp_x = alvo->x;
    s.sp_y = alvo->y;
    if (segurar || !seguidor_.chegou(pos, r_.p.tolerancia_chegada)) return Andamento::EM_CURSO;
    if (seguidor_.eh_ultimo()) {
        seguidor_.avancar();
        return Andamento::CHEGOU;
    }

    const Waypoint* prox = seguidor_.proximo();
    if (prox->passagem >= 0 && !possui(prox->passagem)) {
        auto passagens = seguidor_.passagens_em_sequencia(seguidor_.indice() + 1);
        if (!tentar_passagens(passagens, 1, {r_.cfg.slot})) {
            if (estado == EstadoRobo::DESLOCANDO) mudar_estado(estado, EstadoRobo::AGUARDANDO_PASSAGEM);
            if (agora_ns() - espera_desde_ > int64_t(r_.p.espera_rota_alternativa_s * NS_POR_S)) {
                LOG("T_NAV", "espera longa na passagem %d: pedindo rota alternativa", passagem_espera_);
                passagens_evitadas_.push_back(passagem_espera_);
                desistir_espera();
                pedir_rota(pos, destino_, {}, Finalidade::PROPRIA);
            }
            return Andamento::EM_CURSO;
        }
        if (estado == EstadoRobo::AGUARDANDO_PASSAGEM) mudar_estado(estado, EstadoRobo::DESLOCANDO);
    }
    seguidor_.avancar();
    return Andamento::EM_CURSO;
}

// ------------------------------------------------------------------ passagens

bool Navegacao::possui(int passagem) const {
    return std::any_of(posses_.begin(), posses_.end(), [&](const Posse& p) { return p.passagem == passagem; });
}

bool Navegacao::tentar_passagens(const std::vector<int>& passagens, int k, const std::vector<int>& slots) {
    const int64_t agora = agora_ns();
    const int64_t cessao = int64_t(r_.p.espera_cessao_s * NS_POR_S);
    const int eu = r_.cfg.id;

    // cede a vez a quem espera há mais tempo (e continua tentando), para que as unidades se acumulem
    bool ceder = false;
    for (int p : passagens) {
        auto& esp = r_.frota->passagem(uint32_t(p));
        TravaCompartilhada t(&esp.mtx);
        bool esperador_vivo = agora - esp.ultima_tentativa_ns < 300 * NS_POR_MS;
        if (esp.esperador >= 0 && esp.esperador != eu && esperador_vivo && agora - esp.espera_desde_ns > cessao)
            ceder = true;
    }

    std::vector<SemaforosPassagem::Operacao> ops;
    for (int p : passagens) ops.push_back({p, k});
    bool ok = !ceder && r_.semaforos.tentar_adquirir(ops);

    for (int p : passagens) {
        auto& esp = r_.frota->passagem(uint32_t(p));
        TravaCompartilhada t(&esp.mtx);
        if (ok) {
            esp.ocupacao += k;
            esp.ocupacao_max = std::max(esp.ocupacao_max, esp.ocupacao);
            esp.entradas++;
            for (int i = 0; i < MAX_OCUPANTES; ++i)
                if (esp.ocupantes[i] < 0) {
                    esp.ocupantes[i] = eu;
                    esp.unidades[i] = k;
                    break;
                }
            if (esp.esperador == eu) esp.esperador = -1;
        } else {
            if (esp.esperador < 0 || esp.esperador == eu ||
                agora - esp.ultima_tentativa_ns >= 300 * NS_POR_MS) {
                if (esp.esperador != eu) {
                    esp.espera_desde_ns = agora;
                    esp.esperador_k = k;
                }
                esp.esperador = eu;
                esp.ultima_tentativa_ns = agora;
            }
        }
    }

    if (ok) {
        for (int p : passagens) {
            posses_.push_back({p, k, slots, false});
            LOG("T_NAV", "entrou na passagem %s com %d unidade(s)", r_.cenario.passagens[size_t(p)].nome.c_str(), k);
            char txt[64];
            std::snprintf(txt, sizeof txt, "adquiriu %d vaga(s) em %s", k, r_.cenario.passagens[size_t(p)].nome.c_str());
            r_.registrar(Categoria::ROBO, txt);
        }
        passagem_espera_ = -1;
        espera_avisada_ = false;
        return true;
    }

    if (passagem_espera_ != passagens.front()) {
        passagem_espera_ = passagens.front();
        espera_desde_ = agora;
        espera_avisada_ = false;
    }
    if (!espera_avisada_) {
        espera_avisada_ = true;
        Mensagem m = r_.evento(TipoMsg::PASSAGEM_ESPERA);
        m.evento.extra = passagem_espera_;
        emissor_.enviar(m);
        LOG("T_NAV", "aguardando vaga em %s%s", r_.cenario.passagens[size_t(passagem_espera_)].nome.c_str(),
            ceder ? " (cedendo a vez)" : "");
    }
    return false;
}

void Navegacao::desistir_espera() {
    if (passagem_espera_ < 0) return;
    auto& esp = r_.frota->passagem(uint32_t(passagem_espera_));
    {
        TravaCompartilhada t(&esp.mtx);
        if (esp.esperador == r_.cfg.id) esp.esperador = -1;
    }
    passagem_espera_ = -1;
    espera_avisada_ = false;
}

Ponto Navegacao::posicao_do_slot(int slot) {
    if (slot == r_.cfg.slot) {
        Entradas e = r_.imagem.ler_entradas();
        return {e.x, e.y};
    }
    auto& s = r_.frota->robo(uint32_t(slot));
    TravaCompartilhada t(&s.mtx);
    return {s.x, s.y};
}

// Devolve as vagas quando todos os robôs que entraram estão, pela posição medida, fora da passagem.
void Navegacao::liberar_passagens(bool abandonar_nao_iniciadas) {
    for (auto it = posses_.begin(); it != posses_.end();) {
        bool algum_dentro = false;
        for (int s : it->slots)
            if (r_.mapa.dentro_da_passagem(it->passagem, posicao_do_slot(s), MARGEM_PASSAGEM)) algum_dentro = true;
        if (algum_dentro) it->entrou = true;
        bool liberar = it->entrou ? !algum_dentro : abandonar_nao_iniciadas;
        if (!liberar) {
            ++it;
            continue;
        }
        // o espelho e o registro são atualizados antes de devolver as unidades: assim o espelho nunca
        // mostra mais ocupantes do que o semáforo de fato permite
        {
            auto& esp = r_.frota->passagem(uint32_t(it->passagem));
            TravaCompartilhada t(&esp.mtx);
            esp.ocupacao -= it->unidades;
            for (int i = 0; i < MAX_OCUPANTES; ++i)
                if (esp.ocupantes[i] == r_.cfg.id && esp.unidades[i] == it->unidades) {
                    esp.ocupantes[i] = -1;
                    esp.unidades[i] = 0;
                    break;
                }
        }
        const char* nome = r_.cenario.passagens[size_t(it->passagem)].nome.c_str();
        char txt[64];
        std::snprintf(txt, sizeof txt, "liberou %d vaga(s) em %s", it->unidades, nome);
        r_.registrar(Categoria::ROBO, txt);
        r_.semaforos.liberar(it->passagem, it->unidades);
        LOG("T_NAV", "liberou %d unidade(s) de %s", it->unidades, nome);
        it = posses_.erase(it);
    }
}

// ------------------------------------------------------------------ missões

void Navegacao::iniciar_missao(const Missao& m, const Ponto& pos) {
    {
        std::lock_guard<std::mutex> t(r_.mtx_nav);
        r_.ctx.missao = m;
    }
    objetivo_ = Objetivo::ACOPLAMENTO;
    destino_ = {m.pr.x + m.afastamento.x, m.pr.y + m.afastamento.y};
    atracado_marcado_ = solto_marcado_ = destino_avisado_ = entregue_avisado_ = aborto_avisado_ = false;
    slots_equipe_.clear();
    pedir_rota(pos, destino_, {}, Finalidade::PROPRIA);
    mudar_estado(EstadoRobo::DISPONIVEL, EstadoRobo::DESLOCANDO);
    char txt[96];
    std::snprintf(txt, sizeof txt, "missão: caixa %d, equipe %d, %s", m.caixa, m.equipe,
                  m.papel == Papel::LIDER ? "líder" : "membro");
    r_.registrar(Categoria::ROBO, txt, m.caixa);
}

void Navegacao::iniciar_recarga(const Ponto& pos, EstadoRobo de) {
    if (r_.cenario.estacoes_recarga.empty()) return;
    const Ponto* melhor = &r_.cenario.estacoes_recarga.front();
    for (auto& e : r_.cenario.estacoes_recarga)
        if (distancia(pos, e) < distancia(pos, *melhor)) melhor = &e;
    objetivo_ = Objetivo::RECARGA;
    destino_ = *melhor;
    pedir_rota(pos, destino_, {}, Finalidade::PROPRIA);
    mudar_estado(de, EstadoRobo::DESLOCANDO_RECARGA);
    r_.registrar(Categoria::ROBO, "indo para a estação de recarga");
}

void Navegacao::encerrar_missao(EstadoRobo de, EstadoRobo para) {
    {
        std::lock_guard<std::mutex> t(r_.mtx_nav);
        r_.ctx.missao = Missao{};
    }
    objetivo_ = Objetivo::NENHUM;
    seguidor_.limpar();
    aguardando_rota_ = false;
    slots_equipe_.clear();
    desistir_espera();
    liberar_passagens(true);
    if (de != para) mudar_estado(de, para);
}

SlotEquipe Navegacao::ler_equipe(const Missao& m, bool& valida) {
    SlotEquipe copia{};
    valida = false;
    if (!m.ativa || m.equipe < 0 || uint32_t(m.equipe) >= r_.frota->n_max()) return copia;
    auto& eq = r_.frota->equipe(uint32_t(m.equipe));
    {
        TravaCompartilhada t(&eq.mtx);
        copia = eq;
    }
    valida = copia.epoca == m.epoca && copia.estado != EstadoEquipe::LIVRE;
    return copia;
}

std::vector<int> Navegacao::slots_dos_membros(const SlotEquipe& eq) {
    std::vector<int> slots;
    for (int i = 0; i < eq.k; ++i) {
        for (uint32_t s = 0; s < r_.frota->n_max(); ++s) {
            auto& slot = r_.frota->robo(s);
            TravaCompartilhada t(&slot.mtx);
            if (slot.ocupado && slot.id == eq.membros[i]) {
                slots.push_back(int(s));
                break;
            }
        }
    }
    return slots;
}

void Navegacao::ciclo_equipe(const VisaoContexto& c, const Entradas& e, Saidas& s, bool segurar) {
    bool valida = false;
    SlotEquipe eq = ler_equipe(c.missao, valida);
    const Ponto pos{e.x, e.y};
    const int64_t agora = agora_ns();
    const uint32_t bit = 1u << c.missao.indice;
    const bool abortando = !valida || eq.estado == EstadoEquipe::ABORTANDO;
    auto& slot_eq = r_.frota->equipe(uint32_t(std::max(0, c.missao.equipe)));

    switch (c.estado) {
        case EstadoRobo::AGUARDANDO_EQUIPE:
            s.sp_x = destino_.x;
            s.sp_y = destino_.y;
            if (abortando) {
                encerrar_missao(c.estado, EstadoRobo::DISPONIVEL);
            } else if (eq.estado == EstadoEquipe::REUNIDA && eq.t_go_ns > 0 && agora >= eq.t_go_ns) {
                s.acoplar = true;
                mudar_estado(c.estado, EstadoRobo::ACOPLANDO);
                LOG("T_NAV", "acoplar equipe=%d t_go=%lld t=%lld atraso_ms=%.2f", c.missao.equipe,
                    (long long)eq.t_go_ns, (long long)agora, double(agora - eq.t_go_ns) / NS_POR_MS);
                r_.registrar(Categoria::ROBO, "acionou o_acoplar_e_erguer", c.missao.caixa);
            }
            break;

        case EstadoRobo::ACOPLANDO:
            s.sp_x = destino_.x;
            s.sp_y = destino_.y;
            if (abortando) {
                mudar_estado(c.estado, EstadoRobo::ABORTANDO);
                break;
            }
            if (e.atracada && !atracado_marcado_) {
                {
                    TravaCompartilhada t(&slot_eq.mtx);
                    if (slot_eq.epoca == c.missao.epoca) slot_eq.mascara_atracado |= bit;
                }
                atracado_marcado_ = true;
                Mensagem m = r_.evento(TipoMsg::ATRACADO);
                m.evento.equipe = c.missao.equipe;
                m.evento.caixa = c.missao.caixa;
                emissor_.enviar(m);
            }
            if (eq.estado == EstadoEquipe::TRANSPORTE) {
                mudar_estado(c.estado, EstadoRobo::TRANSPORTANDO);
                if (c.missao.papel == Papel::LIDER) {
                    slots_equipe_ = slots_dos_membros(eq);
                    OpcoesRota op;
                    op.k = eq.k;
                    pedir_rota(c.missao.pr, c.missao.zona, op, Finalidade::CARGA);
                }
            }
            break;

        case EstadoRobo::TRANSPORTANDO:
            if (abortando)
                mudar_estado(c.estado, EstadoRobo::ABORTANDO);
            else
                transportar(c, eq, pos, s, segurar);
            break;

        case EstadoRobo::DESACOPLANDO:
            if (!e.atracada && !solto_marcado_) {
                TravaCompartilhada t(&slot_eq.mtx);
                if (slot_eq.epoca == c.missao.epoca) slot_eq.mascara_atracado &= ~bit;
                solto_marcado_ = true;
            }
            if (c.missao.papel == Papel::LIDER && !entregue_avisado_ && valida && eq.mascara_atracado == 0) {
                entregue_avisado_ = true;
                Mensagem m = r_.evento(TipoMsg::ENTREGUE);
                m.evento.equipe = c.missao.equipe;
                m.evento.caixa = c.missao.caixa;
                m.evento.x = float(pos.x);
                m.evento.y = float(pos.y);
                emissor_.enviar(m);
            }
            if (!valida || eq.estado == EstadoEquipe::CONCLUIDA) encerrar_missao(c.estado, EstadoRobo::DISPONIVEL);
            break;

        case EstadoRobo::ABORTANDO:
            s.sp_x = pos.x;
            s.sp_y = pos.y;
            if (!valida || (eq.t_go_ns > 0 && agora >= eq.t_go_ns)) s.acoplar = false;
            if (!s.acoplar && !e.atracada && !aborto_avisado_) {
                aborto_avisado_ = true;
                Mensagem m = r_.evento(TipoMsg::ABORTO_CONCLUIDO);
                m.evento.equipe = c.missao.equipe;
                m.evento.caixa = c.missao.caixa;
                m.evento.x = float(pos.x);
                m.evento.y = float(pos.y);
                emissor_.enviar(m);
                r_.registrar(Categoria::ROBO, "carga depositada no local (aborto)", c.missao.caixa);
                encerrar_missao(c.estado, EstadoRobo::DISPONIVEL);
            }
            break;

        default: break;
    }
}

void Navegacao::transportar(const VisaoContexto& c, const SlotEquipe& eq, const Ponto& pos, Saidas& s, bool segurar) {
    const int64_t agora = agora_ns();
    auto& slot_eq = r_.frota->equipe(uint32_t(c.missao.equipe));
    const bool lider = c.missao.papel == Papel::LIDER;

    if (eq.estado == EstadoEquipe::DESACOPLANDO && eq.t_go_ns > 0 && agora >= eq.t_go_ns) {
        s.acoplar = false;
        mudar_estado(EstadoRobo::TRANSPORTANDO, EstadoRobo::DESACOPLANDO);
        LOG("T_NAV", "desacoplar equipe=%d atraso_ms=%.2f", c.missao.equipe, double(agora - eq.t_go_ns) / NS_POR_MS);
        return;
    }

    if (eq.rota_n == 0) {
        s.sp_x = destino_.x;
        s.sp_y = destino_.y;
        if (!lider || !aguardando_rota_) return;
        Rota rota;
        bool ok = false;
        if (!verificar_rota(pos, rota, ok)) return;
        if (!ok || rota.size() > size_t(MAX_ROTA)) {
            Mensagem m = r_.evento(TipoMsg::SEM_ROTA);
            m.evento.equipe = c.missao.equipe;
            emissor_.enviar(m);
            abortar_equipe(slot_eq, c.missao.epoca);
            return;
        }
        TravaCompartilhada t(&slot_eq.mtx);
        if (slot_eq.epoca != c.missao.epoca) return;
        for (size_t i = 0; i < rota.size(); ++i) {
            slot_eq.rota_x[i] = float(rota[i].x);
            slot_eq.rota_y[i] = float(rota[i].y);
            slot_eq.rota_passagem[i] = int8_t(rota[i].passagem);
        }
        slot_eq.rota_n = int32_t(rota.size());
        slot_eq.rota_versao++;
        slot_eq.passo = 0;
        slot_eq.mascara_chegada = 0;
        return;
    }

    const int passo = eq.passo;
    const Ponto alvo{eq.rota_x[passo] + c.missao.afastamento.x, eq.rota_y[passo] + c.missao.afastamento.y};
    s.sp_x = alvo.x;
    s.sp_y = alvo.y;
    const uint32_t bit = 1u << c.missao.indice;
    if (!segurar && !(eq.mascara_chegada & bit) && distancia(pos, alvo) <= r_.p.tolerancia_chegada) {
        TravaCompartilhada t(&slot_eq.mtx);
        if (slot_eq.epoca == c.missao.epoca && slot_eq.passo == passo) slot_eq.mascara_chegada |= bit;
    }
    if (!lider) return;

    const uint32_t completa = (1u << eq.k) - 1;
    if ((eq.mascara_chegada & completa) != completa) return;

    if (passo == eq.rota_n - 1) {
        if (!destino_avisado_) {
            destino_avisado_ = true;
            Mensagem m = r_.evento(TipoMsg::EM_POSICAO);
            m.evento.equipe = c.missao.equipe;
            m.evento.caixa = c.missao.caixa;
            m.evento.fase = Fase::DESTINO;
            emissor_.enviar(m);
        }
        return;
    }

    const int prox = passo + 1;
    if (eq.rota_passagem[prox] >= 0 && !possui(eq.rota_passagem[prox])) {
        std::vector<int> passagens;
        for (int i = prox; i < eq.rota_n && eq.rota_passagem[i] >= 0; ++i)
            if (passagens.empty() || passagens.back() != eq.rota_passagem[i]) passagens.push_back(eq.rota_passagem[i]);
        if (slots_equipe_.empty()) slots_equipe_ = slots_dos_membros(eq);
        if (!tentar_passagens(passagens, eq.k, slots_equipe_)) return;  // a equipe espera no passo atual
    }
    TravaCompartilhada t(&slot_eq.mtx);
    if (slot_eq.epoca == c.missao.epoca && slot_eq.passo == passo) {
        slot_eq.passo = prox;
        slot_eq.mascara_chegada = 0;
    }
}

// ------------------------------------------------------------------ ciclo

void Navegacao::ciclo(const InfoCiclo& info) {
    emissor_.retentar();
    const Entradas e = r_.imagem.ler_entradas();
    const Ponto pos{e.x, e.y};

    processar_comandos(ler_contexto(), pos);
    const VisaoContexto c = ler_contexto();
    if (c.encerrar) return;
    avaliar_prazo(info);

    const bool parada = r_.frota->cabecalho().parada_geral.load() != 0;
    const bool segurar = parada || c.inibido || c.parado || c.sem_comunicacao || congelado_prazo_;
    if (segurar && !segurando_)
        LOG("T_NAV", "setpoint congelado (%s) t_mono=%lld",
            parada ? "parada geral" : c.inibido ? "defeito" : c.parado ? "PARAR" : c.sem_comunicacao ? "sem comunicação" : "prazo",
            (long long)agora_ns());
    segurando_ = segurar;

    liberar_passagens(false);
    Saidas s = saida_;

    switch (c.estado) {
        case EstadoRobo::DISPONIVEL:
            s.sp_x = pos.x;
            s.sp_y = pos.y;
            if (c.recarga_pendente && !segurar) iniciar_recarga(pos, c.estado);
            break;

        case EstadoRobo::DEFEITO:
            s.sp_x = pos.x;
            s.sp_y = pos.y;
            if (c.missao.ativa) {
                bool valida = false;
                SlotEquipe eq = ler_equipe(c.missao, valida);
                if (!valida || (eq.t_go_ns > 0 && agora_ns() >= eq.t_go_ns && eq.estado == EstadoEquipe::ABORTANDO))
                    s.acoplar = false;
                if (!s.acoplar && !e.atracada) encerrar_missao(c.estado, c.estado);
            }
            break;

        case EstadoRobo::DESLOCANDO:
        case EstadoRobo::AGUARDANDO_PASSAGEM:
        case EstadoRobo::DESLOCANDO_RECARGA: {
            if (objetivo_ == Objetivo::ACOPLAMENTO) {
                bool valida = false;
                SlotEquipe eq = ler_equipe(c.missao, valida);
                if (!valida || eq.estado == EstadoEquipe::ABORTANDO) {
                    encerrar_missao(c.estado, EstadoRobo::DISPONIVEL);
                    break;
                }
            }
            if (objetivo_ == Objetivo::MANUAL && c.bateria_critica) {
                iniciar_recarga(pos, c.estado);
                break;
            }
            Andamento a = seguir(pos, s, segurar, c.estado);
            EstadoRobo atual = c.estado == EstadoRobo::DESLOCANDO_RECARGA ? c.estado : ler_contexto().estado;
            if (a == Andamento::CHEGOU) {
                if (objetivo_ == Objetivo::ACOPLAMENTO) {
                    Mensagem m = r_.evento(TipoMsg::EM_POSICAO);
                    m.evento.equipe = c.missao.equipe;
                    m.evento.caixa = c.missao.caixa;
                    m.evento.fase = Fase::ACOPLAMENTO;
                    m.evento.x = float(pos.x);
                    m.evento.y = float(pos.y);
                    emissor_.enviar(m);
                    mudar_estado(atual, EstadoRobo::AGUARDANDO_EQUIPE);
                } else {
                    mudar_estado(atual, objetivo_ == Objetivo::RECARGA ? EstadoRobo::RECARREGANDO : EstadoRobo::DISPONIVEL);
                    objetivo_ = Objetivo::NENHUM;
                }
            } else if (a == Andamento::FALHOU) {
                if (objetivo_ == Objetivo::ACOPLAMENTO)
                    abortar_equipe(r_.frota->equipe(uint32_t(c.missao.equipe)), c.missao.epoca);
                encerrar_missao(atual, EstadoRobo::DISPONIVEL);
            }
            break;
        }

        default:
            if (estado_de_equipe(c.estado))
                ciclo_equipe(c, e, s, segurar);
            else {
                s.sp_x = pos.x;
                s.sp_y = pos.y;
            }
            break;
    }

    if (segurar) {
        s.sp_x = pos.x;
        s.sp_y = pos.y;
    }
    saida_ = s;
    r_.imagem.escrever_saidas(s);
}

}  // namespace amr
