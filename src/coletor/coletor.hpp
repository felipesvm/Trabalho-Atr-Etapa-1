#pragma once

#include <atomic>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "../comum/eventos.hpp"
#include "../comum/frota.hpp"
#include "../comum/ipc.hpp"
#include "../comum/tarefa_periodica.hpp"

namespace amr {

// Toda a escrita em disco fica neste processo, longe das threads de tempo real.
class Coletor {
public:
    explicit Coletor(const std::string& raiz_logs);
    ~Coletor();

    void iniciar();
    void parar();
    const std::string& pasta() const { return pasta_; }

private:
    void laco_consumo();
    void amostrar_frota();
    void gravar(const Registro& r);
    void sincronizar();

    std::string pasta_;
    MemoriaCompartilhada shm_frota_;
    MemoriaCompartilhada shm_eventos_;
    std::unique_ptr<Frota> frota_;
    BufferEventos* buffer_ = nullptr;
    SemaforoNomeado vazios_, itens_;

    std::FILE* eventos_ = nullptr;
    std::FILE* cargas_ = nullptr;
    std::FILE* amostras_ = nullptr;

    std::atomic<bool> ativo_{false};
    std::thread t_cons_;
    std::unique_ptr<TarefaPeriodica> t_snap_;
};

}  // namespace amr
