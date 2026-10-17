#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "../comum/cenario.hpp"
#include "../comum/eventos.hpp"
#include "../comum/frota.hpp"
#include "../comum/ipc.hpp"
#include "../comum/tarefa_periodica.hpp"
#include "despachante.hpp"
#include "modelo.hpp"

namespace amr {

// Dono dos objetos de IPC. Três threads: eventos (T_ORG_EVT), despacho (T_ORG_DESP) e supervisão (T_ORG_SUP).
class Organizador {
public:
    Organizador(Cenario cenario, uint32_t n_max);
    ~Organizador();

    void iniciar();
    void parar();

private:
    struct FotoSlot {
        bool ocupado;
        int id;
        uint32_t epoca;
        EstadoRobo estado;
        ModoOperacao modo;
        Ponto pos;
        bool bateria_baixa, falha, parado;
        uint64_t heartbeat;
        bool suspeito;
    };

    void criar_ipc();
    void remover_ipc();
    std::vector<FotoSlot> fotografar_frota();

    void laco_eventos();
    void tratar(const Mensagem& m, Efeitos& ef);
    void tratar_operador(const CorpoOperador& op, Efeitos& ef);
    void ciclo_despacho();
    void ciclo_supervisao();

    void abortar_equipe(int indice, const char* motivo, Efeitos& ef);
    void liberar_equipe(int indice, Efeitos& ef);
    void enviar_robo(int robo, const Mensagem& m, Efeitos& ef);
    void aplicar(Efeitos& ef);
    void abastecer_pontos_de_retirada();
    void registrar(const Registro& r) { rastreador_->registrar(r); }
    Registro registro(Categoria cat, const char* formato, ...) __attribute__((format(printf, 3, 4)));

    Cenario cenario_;
    uint32_t n_max_;
    Despachante despachante_;

    MemoriaCompartilhada shm_frota_;
    MemoriaCompartilhada shm_eventos_;
    std::unique_ptr<Frota> frota_;
    SemaforosPassagem semaforos_;
    SemaforoNomeado vazios_, itens_;
    FilaMensagens entrada_;
    std::unique_ptr<Rastreador> rastreador_;

    std::mutex mtx_modelo_;
    Modelo modelo_;

    std::atomic<bool> ativo_{false};
    std::thread t_evt_;
    std::unique_ptr<TarefaPeriodica> t_desp_;
    std::unique_ptr<TarefaPeriodica> t_sup_;
    int64_t ultimo_resumo_ = 0;
    int ultima_reserva_ = -1;
    std::vector<int> esperas_avisadas_;
};

}  // namespace amr
