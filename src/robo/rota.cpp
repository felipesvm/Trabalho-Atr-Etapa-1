#include "rota.hpp"

#include <chrono>

#include "../comum/console.hpp"
#include "../comum/tarefa_periodica.hpp"

namespace amr {

void executar_planejador(NucleoRobo& r) {
    aplicar_prioridade(40, "T_ROTA");
    uint32_t atendida = 0;
    std::unique_lock<std::mutex> trava(r.mtx_nav);
    while (r.ativo.load()) {
        r.cv_rota.wait_for(trava, std::chrono::milliseconds(200),
                           [&] { return r.ctx.pedido.versao > atendida || !r.ativo.load(); });
        if (!r.ativo.load()) break;
        if (r.ctx.pedido.versao <= atendida) continue;

        PedidoRota pedido = r.ctx.pedido;
        trava.unlock();
        auto rota = r.mapa.planejar(pedido.origem, pedido.destino, pedido.opcoes);
        trava.lock();

        atendida = pedido.versao;
        if (r.ctx.pedido.versao != pedido.versao) continue;
        r.ctx.planejada.versao = pedido.versao;
        r.ctx.planejada.ok = rota.has_value();
        r.ctx.planejada.rota = rota ? std::move(*rota) : Rota{};
    }
}

}  // namespace amr
