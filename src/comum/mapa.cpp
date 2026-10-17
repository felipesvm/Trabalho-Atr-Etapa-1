#include "mapa.hpp"

#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdlib>
#include <queue>

namespace amr {

Mapa::Mapa(const Cenario& c)
    : largura_(c.largura),
      altura_(c.altura),
      bloqueio_(size_t(c.largura * c.altura), 0),
      passagem_(size_t(c.largura * c.altura), -1),
      passagens_(c.passagens) {
    for (auto& r : c.obstaculos)
        for (int y = r.y; y < r.y + r.h; ++y)
            for (int x = r.x; x < r.x + r.w; ++x)
                if (x >= 0 && y >= 0 && x < largura_ && y < altura_) bloqueio_[size_t(indice(x, y))] = 1;
    for (size_t i = 0; i < passagens_.size(); ++i) {
        auto& r = passagens_[i].area;
        for (int y = r.y; y < r.y + r.h; ++y)
            for (int x = r.x; x < r.x + r.w; ++x)
                if (x >= 0 && y >= 0 && x < largura_ && y < altura_) {
                    bloqueio_[size_t(indice(x, y))] = 0;
                    passagem_[size_t(indice(x, y))] = int(i);
                }
    }
}

bool Mapa::livre(int cx, int cy) const {
    return cx >= 0 && cy >= 0 && cx < largura_ && cy < altura_ && !bloqueio_[size_t(indice(cx, cy))];
}

int Mapa::passagem_da_celula(int cx, int cy) const {
    if (cx < 0 || cy < 0 || cx >= largura_ || cy >= altura_) return -1;
    return passagem_[size_t(indice(cx, cy))];
}

bool Mapa::dentro_da_passagem(int p, const Ponto& pt, double margem) const {
    const auto& r = area(p);
    return pt.x > r.x - margem && pt.x < r.x + r.w + margem && pt.y > r.y - margem && pt.y < r.y + r.h + margem;
}

bool Mapa::transitavel(int cx, int cy, const OpcoesRota& op) const {
    if (!livre(cx, cy)) return false;
    int p = passagem_da_celula(cx, cy);
    if (p < 0) return true;
    if (op.somente_corredores) return false;
    if (passagens_[size_t(p)].capacidade < op.k) return false;
    return std::find(op.bloqueadas.begin(), op.bloqueadas.end(), p) == op.bloqueadas.end();
}

std::optional<Rota> Mapa::planejar(const Ponto& origem, const Ponto& destino, const OpcoesRota& op) const {
    int ox = int(std::floor(origem.x)), oy = int(std::floor(origem.y));
    int dx = int(std::floor(destino.x)), dy = int(std::floor(destino.y));
    if (!livre(ox, oy) || !livre(dx, dy)) return std::nullopt;
    if (!transitavel(dx, dy, op)) return std::nullopt;

    const int n = largura_ * altura_;
    std::vector<int> custo(size_t(n), INT_MAX), anterior(size_t(n), -1);
    using Item = std::pair<int, int>;  // (f, célula)
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> abertos;
    auto h = [&](int x, int y) { return std::abs(x - dx) + std::abs(y - dy); };

    int inicio = indice(ox, oy), alvo = indice(dx, dy);
    custo[size_t(inicio)] = 0;
    abertos.push({h(ox, oy), inicio});
    const int vx[] = {1, -1, 0, 0}, vy[] = {0, 0, 1, -1};

    while (!abertos.empty()) {
        auto [f, atual] = abertos.top();
        abertos.pop();
        if (atual == alvo) break;
        int cx = atual % largura_, cy = atual / largura_;
        if (f - h(cx, cy) > custo[size_t(atual)]) continue;
        for (int d = 0; d < 4; ++d) {
            int nx = cx + vx[d], ny = cy + vy[d];
            if (!transitavel(nx, ny, op)) continue;
            int viz = indice(nx, ny);
            int c = custo[size_t(atual)] + 1;
            if (c < custo[size_t(viz)]) {
                custo[size_t(viz)] = c;
                anterior[size_t(viz)] = atual;
                abertos.push({c + h(nx, ny), viz});
            }
        }
    }
    if (inicio != alvo && anterior[size_t(alvo)] < 0) return std::nullopt;

    std::vector<int> celulas;
    for (int c = alvo; c != inicio; c = anterior[size_t(c)]) celulas.push_back(c);
    std::reverse(celulas.begin(), celulas.end());

    Rota rota;
    for (int c : celulas) {
        int cx = c % largura_, cy = c / largura_;
        rota.push_back({cx + 0.5, cy + 0.5, passagem_da_celula(cx, cy)});
    }
    if (rota.empty())
        rota.push_back({destino.x, destino.y, passagem_da_celula(dx, dy)});
    else
        rota.back() = {destino.x, destino.y, rota.back().passagem};
    return rota;
}

}  // namespace amr
