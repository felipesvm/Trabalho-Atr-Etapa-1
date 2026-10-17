#pragma once

#include <optional>
#include <vector>

#include "cenario.hpp"

namespace amr {

struct Waypoint {
    double x = 0, y = 0;
    int passagem = -1;  // índice da passagem que contém o ponto, ou -1
};

using Rota = std::vector<Waypoint>;

struct OpcoesRota {
    int k = 1;                       // robôs que atravessam juntos (equipe acoplada)
    std::vector<int> bloqueadas;     // passagens a evitar (rota alternativa)
    bool somente_corredores = false; // heurística simplificada: nenhuma passagem estreita
};

// Grade de células de 1 m. Passagens são células livres marcadas com o índice da passagem.
class Mapa {
public:
    explicit Mapa(const Cenario& cenario);

    int largura() const { return largura_; }
    int altura() const { return altura_; }
    bool livre(int cx, int cy) const;
    int passagem_da_celula(int cx, int cy) const;
    bool dentro_da_passagem(int passagem, const Ponto& p, double margem) const;
    const Retangulo& area(int passagem) const { return passagens_[size_t(passagem)].area; }
    int capacidade(int passagem) const { return passagens_[size_t(passagem)].capacidade; }
    int quantidade_passagens() const { return int(passagens_.size()); }

    // A* em 4 vizinhos. A rota começa na primeira célula após a origem e termina no destino exato.
    std::optional<Rota> planejar(const Ponto& origem, const Ponto& destino, const OpcoesRota& opcoes) const;

private:
    int indice(int cx, int cy) const { return cy * largura_ + cx; }
    bool transitavel(int cx, int cy, const OpcoesRota& opcoes) const;

    int largura_, altura_;
    std::vector<char> bloqueio_;
    std::vector<int> passagem_;
    std::vector<DefPassagem> passagens_;
};

}  // namespace amr
