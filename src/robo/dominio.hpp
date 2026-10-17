#pragma once

// Classes de domínio do robô: sem threads, sem IPC, testáveis isoladamente.

#include <cstddef>
#include <vector>

#include "../comum/cenario.hpp"
#include "../comum/mapa.hpp"

namespace amr {

class SeguidorRota {
public:
    void definir(Rota rota) {
        rota_ = std::move(rota);
        indice_ = 0;
    }
    void limpar() { definir({}); }

    bool vazia() const { return rota_.empty(); }
    bool concluida() const { return indice_ >= rota_.size(); }
    size_t indice() const { return indice_; }
    const Waypoint* alvo() const { return concluida() ? nullptr : &rota_[indice_]; }
    const Waypoint* proximo() const { return indice_ + 1 < rota_.size() ? &rota_[indice_ + 1] : nullptr; }
    bool eh_ultimo() const { return indice_ + 1 == rota_.size(); }
    void avancar() {
        if (!concluida()) ++indice_;
    }

    bool chegou(const Ponto& pos, double tolerancia) const {
        auto* a = alvo();
        return a && distancia(pos, {a->x, a->y}) <= tolerancia;
    }

    // Passagens consecutivas a partir de i, até a primeira célula livre:
    // precisam ser adquiridas juntas para não segurar uma esperando a outra.
    std::vector<int> passagens_em_sequencia(size_t i) const {
        std::vector<int> v;
        for (; i < rota_.size() && rota_[i].passagem >= 0; ++i)
            if (v.empty() || v.back() != rota_[i].passagem) v.push_back(rota_[i].passagem);
        return v;
    }

    const Rota& rota() const { return rota_; }

private:
    Rota rota_;
    size_t indice_ = 0;
};

enum class NivelBateria { NORMAL, BAIXA, CRITICA };

// Avalia o nível da bateria com histerese: só volta a NORMAL depois de recarregada.
class MonitorBateria {
public:
    MonitorBateria(double baixa, double critica, double recarregada)
        : baixa_(baixa), critica_(critica), recarregada_(recarregada) {}

    NivelBateria nivel() const { return nivel_; }

    // Retorna verdadeiro quando o nível mudou nesta avaliação.
    bool avaliar(double carga) {
        NivelBateria novo = nivel_;
        if (carga <= critica_)
            novo = NivelBateria::CRITICA;
        else if (carga <= baixa_ && nivel_ == NivelBateria::NORMAL)
            novo = NivelBateria::BAIXA;
        else if (carga >= recarregada_)
            novo = NivelBateria::NORMAL;
        bool mudou = novo != nivel_;
        nivel_ = novo;
        return mudou;
    }

private:
    double baixa_, critica_, recarregada_;
    NivelBateria nivel_ = NivelBateria::NORMAL;
};

class DetectorBorda {
public:
    // Retorna +1 na borda de subida, -1 na de descida e 0 sem mudança.
    int atualizar(bool valor) {
        int borda = valor == anterior_ ? 0 : (valor ? 1 : -1);
        anterior_ = valor;
        return borda;
    }

private:
    bool anterior_ = false;
};

// Afastamento de cada membro em relação ao ponto da carga; o membro 0 é o líder.
inline Ponto afastamento_formacao(int k, int indice, double d) {
    switch (k) {
        case 2: return indice == 0 ? Ponto{-d, 0} : Ponto{d, 0};
        case 3: {
            const Ponto v[] = {{-d, -d}, {d, -d}, {0, d}};
            return v[indice % 3];
        }
        case 4: {
            const Ponto v[] = {{-d, -d}, {d, -d}, {-d, d}, {d, d}};
            return v[indice % 4];
        }
        default: return {0, 0};
    }
}

}  // namespace amr
