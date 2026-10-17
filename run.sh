#!/usr/bin/env bash
# Inicia o sistema da Etapa 1 a partir da raiz do repositório e encerra tudo com Ctrl+C.
#   ./run.sh [--robos N] [--n-max M] [--cenario arquivo] [--sem-rt] [--bin pasta]

set -euo pipefail
cd "$(dirname "$0")"

ROBOS=4
N_MAX=32
CENARIO=config/cenario.json
BIN=bin

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robos)   ROBOS="$2"; shift 2 ;;
        --n-max)   N_MAX="$2"; shift 2 ;;
        --cenario) CENARIO="$2"; shift 2 ;;
        --bin)     BIN="$2"; shift 2 ;;
        --sem-rt)  export AMR_SEM_RT=1; shift ;;
        -h|--help) sed -n '2,3p' "$0"; exit 0 ;;
        *) echo "opção desconhecida: $1" >&2; exit 2 ;;
    esac
done

if [[ ! -x "$BIN/amr_organizador" ]]; then
    echo "binários não encontrados em $BIN/: rode 'make' antes" >&2
    exit 1
fi
if (( ROBOS > N_MAX )); then
    echo "--robos ($ROBOS) maior que --n-max ($N_MAX)" >&2
    exit 2
fi

# filas maiores que o padrão do Linux (10 mensagens), se houver permissão
if [[ -w /proc/sys/fs/mqueue/msg_max ]] && (( $(cat /proc/sys/fs/mqueue/msg_max) < 64 )); then
    echo 64 > /proc/sys/fs/mqueue/msg_max || true
fi

PIDS=()
encerrar() {
    trap - INT TERM
    echo
    echo "encerrando: gestor (e robôs), coletor, organizador"
    for nome in amr_gestor amr_coletor amr_organizador; do
        for pid in "${PIDS[@]}"; do
            if [[ "$(ps -o comm= -p "$pid" 2>/dev/null)" == "$nome" ]]; then
                kill -TERM "$pid" 2>/dev/null || true
                wait "$pid" 2>/dev/null || true
            fi
        done
    done
    "$BIN/limpar_ipc" > /dev/null
    exit 0
}
trap encerrar INT TERM

"$BIN/limpar_ipc" > /dev/null

# cada processo em sua própria sessão: o Ctrl+C chega só a este script, que encerra tudo na ordem
setsid "$BIN/amr_organizador" --cenario "$CENARIO" --n-max "$N_MAX" &
PIDS+=($!)
for _ in $(seq 50); do
    [[ -e /dev/shm/amr_frota ]] && break
    sleep 0.1
done

setsid "$BIN/amr_coletor" --logs logs &
PIDS+=($!)
setsid "$BIN/amr_gestor" --cenario "$CENARIO" &
PIDS+=($!)
sleep 0.5

if (( ROBOS > 0 )); then
    "$BIN/anunciar_robo" --n "$ROBOS" --cenario "$CENARIO" > /dev/null
fi
echo "sistema em execução com $ROBOS robô(s); acompanhe com $BIN/amr_monitor em outro terminal (Ctrl+C encerra)"

while true; do
    sleep 1 &
    wait $! || true
done
