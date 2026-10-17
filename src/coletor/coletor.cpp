#include "coletor.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "../comum/console.hpp"
#include "../comum/nomes.hpp"
#include "../comum/tempo.hpp"

namespace amr {

namespace {
constexpr int LOTE_MAX = 64;

std::FILE* abrir_csv(const std::string& caminho, const char* cabecalho) {
    std::FILE* f = std::fopen(caminho.c_str(), "w");
    if (!f) throw std::runtime_error("não foi possível criar " + caminho + ": " + std::strerror(errno));
    std::fputs(cabecalho, f);
    return f;
}

std::string pasta_da_execucao(const std::string& raiz) {
    mkdir(raiz.c_str(), 0775);
    time_t agora = time(nullptr);
    tm local{};
    localtime_r(&agora, &local);
    char nome[32];
    std::strftime(nome, sizeof nome, "%Y%m%d-%H%M%S", &local);
    std::string pasta = raiz + "/" + nome;
    for (int i = 1; mkdir(pasta.c_str(), 0775) != 0; ++i) {
        if (errno != EEXIST) throw std::runtime_error("não foi possível criar " + pasta);
        pasta = raiz + "/" + nome + "-" + std::to_string(i);
    }
    std::string atalho = raiz + "/ultima";
    unlink(atalho.c_str());
    // o atalho logs/ultima é só conveniência; se o sistema de arquivos não permitir, segue sem ele
    if (symlink(pasta.substr(raiz.size() + 1).c_str(), atalho.c_str()) != 0) LOG("main", "sem atalho %s", atalho.c_str());
    return pasta;
}

std::string lista_robos(const Registro& r) {
    std::string s;
    for (int id : r.robos)
        if (id >= 0) s += (s.empty() ? "" : ",") + std::to_string(id);
    return s;
}
}  // namespace

Coletor::Coletor(const std::string& raiz_logs) {
    shm_frota_ = MemoriaCompartilhada::abrir(nomes::SHM_FROTA);
    frota_ = std::make_unique<Frota>(shm_frota_.dados());
    shm_eventos_ = MemoriaCompartilhada::abrir(nomes::SHM_EVENTOS);
    buffer_ = static_cast<BufferEventos*>(shm_eventos_.dados());
    vazios_ = SemaforoNomeado::abrir(nomes::SEM_VAZIOS);
    itens_ = SemaforoNomeado::abrir(nomes::SEM_ITENS);

    pasta_ = pasta_da_execucao(raiz_logs);
    const char* cab = "data_hora;origem;categoria;robo;caixa;baia;zona;robos;texto;t_mono_ns\n";
    eventos_ = abrir_csv(pasta_ + "/eventos.csv", cab);
    cargas_ = abrir_csv(pasta_ + "/cargas.csv", cab);
    amostras_ = abrir_csv(pasta_ + "/frota.csv",
                          "data_hora;slot;id;pid;estado;modo;x;y;bateria;distancia;equipe;caixa;heartbeat;"
                          "nav_ciclos;nav_perdas;nav_exec_max_us;nav_atraso_max_us;"
                          "status_ciclos;status_perdas;status_exec_max_us;status_atraso_max_us\n");
}

Coletor::~Coletor() {
    parar();
    for (auto* f : {eventos_, cargas_, amostras_})
        if (f) std::fclose(f);
}

void Coletor::iniciar() {
    ativo_ = true;
    t_cons_ = std::thread(&Coletor::laco_consumo, this);
    t_snap_ = std::make_unique<TarefaPeriodica>("T_COL_SNAP", NS_POR_S, 0, [this](const InfoCiclo&) { amostrar_frota(); });
    t_snap_->iniciar();
}

void Coletor::parar() {
    if (!ativo_.exchange(false)) return;
    if (t_cons_.joinable()) t_cons_.join();
    if (t_snap_) t_snap_->parar();
    sincronizar();
}

void Coletor::gravar(const Registro& r) {
    char linha[512];
    std::snprintf(linha, sizeof linha, "%s;%s;%s;%d;%d;%d;%d;%s;%s;%lld\n", data_hora_iso(r.t_parede_ns).c_str(), r.origem,
                  nome_categoria(r.categoria), r.robo, r.caixa, r.baia, r.zona, lista_robos(r).c_str(), r.texto,
                  (long long)r.t_mono_ns);
    std::fputs(linha, eventos_);
    if (r.categoria == Categoria::CARGA) std::fputs(linha, cargas_);
}

void Coletor::sincronizar() {
    for (auto* f : {eventos_, cargas_, amostras_}) {
        if (!f) continue;
        std::fflush(f);
        fsync(fileno(f));
    }
}

// T_COL_CONS: consumidor do produtor–consumidor (sem_itens / sem_vazios / mutex do buffer)
void Coletor::laco_consumo() {
    pthread_setname_np(pthread_self(), "T_COL_CONS");
    Registro lote[LOTE_MAX];
    int64_t ultimo_sync = agora_ns();
    while (ativo_.load()) {
        int n = 0;
        if (itens_.esperar_por(NS_POR_S)) {
            n = 1;
            while (n < LOTE_MAX && itens_.tentar()) ++n;
            {
                TravaCompartilhada t(&buffer_->mtx);
                for (int i = 0; i < n; ++i) {
                    lote[i] = buffer_->itens[buffer_->cabeca];
                    buffer_->cabeca = (buffer_->cabeca + 1) % buffer_->capacidade;
                }
            }
            for (int i = 0; i < n; ++i) vazios_.liberar();
            for (int i = 0; i < n; ++i) gravar(lote[i]);
            buffer_->gravados.fetch_add(uint64_t(n));
        }
        if (agora_ns() - ultimo_sync >= NS_POR_S) {
            sincronizar();
            ultimo_sync = agora_ns();
        }
    }
    // esvazia o que restou antes de sair
    while (itens_.tentar()) {
        Registro r;
        {
            TravaCompartilhada t(&buffer_->mtx);
            r = buffer_->itens[buffer_->cabeca];
            buffer_->cabeca = (buffer_->cabeca + 1) % buffer_->capacidade;
        }
        vazios_.liberar();
        gravar(r);
    }
}

// T_COL_SNAP
void Coletor::amostrar_frota() {
    std::string hora = data_hora_iso(parede_ns());
    for (uint32_t s = 0; s < frota_->n_max(); ++s) {
        SlotRobo c;
        {
            auto& sl = frota_->robo(s);
            TravaCompartilhada t(&sl.mtx);
            if (t.dono_morreu()) LOG("T_COL_SNAP", "EOWNERDEAD no slot %u: amostra marcada como suspeita", s);
            if (!sl.ocupado) continue;
            c = sl;
        }
        std::fprintf(amostras_, "%s;%u;%d;%d;%s;%s;%.3f;%.3f;%.2f;%.3f;%d;%d;%llu;%llu;%llu;%u;%u;%llu;%llu;%u;%u\n",
                     hora.c_str(), s, c.id, c.pid, nome_estado(c.estado), c.modo == ModoOperacao::AUTO ? "AUTO" : "MANUAL",
                     c.x, c.y, c.bateria, c.distancia, c.equipe, c.caixa, (unsigned long long)c.heartbeat,
                     (unsigned long long)c.nav.ciclos, (unsigned long long)c.nav.perdas_prazo, c.nav.execucao_max_us,
                     c.nav.atraso_max_us, (unsigned long long)c.status.ciclos, (unsigned long long)c.status.perdas_prazo,
                     c.status.execucao_max_us, c.status.atraso_max_us);
    }
}

}  // namespace amr
