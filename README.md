# EMCUA
Embedded DC Motor Controller Using AVR

![EMCUA Logo](assets/logo.png)

Projeto de controle de motor DC embarcado (AVR). O repositório contém o firmware para controlar um motor DC em malha aberta e fechada, além da interface UART para receber comandos.

## Circuito (Datapool)

A imagem abaixo mostra o circuito e a placa usada para testes (Datapool). Salve a foto em `assets/datapool.jpg` para que o link funcione no README.

![Datapool Circuit](assets/datapool.jpg)

## Configuração UART

Use a interface serial para enviar comandos ASCII terminados por CR ou LF (carriage return ou newline). A inicialização padrão do UART no código usa `uartInit(baud)`; o firmware assume tipicamente `115200 8N1` mas você pode configurar outro baud chamando `uartInit()` no `main.c`.

## Comandos seriais suportados

O firmware implementa os seguintes comandos (enviados como texto, sem CR/LF — estes apenas terminam a linha):

- `SSC.1234` — Set Speed Closed-loop
	- Formato: `SSC.` seguido por exatamente 4 dígitos decimais (0-9).
	- Ação: ajusta a referência de RPM do controle em malha fechada.
	- Exemplo: `SSC.0450` define referência para 450 RPM.

- `SCG.XX` — Set GAIN (multiplicador da ação proporcional)
	- Formato: `SCG.` seguido por 1 a 2 dígitos decimais.
	- Ação: define o valor inteiro de `GAIN` usado na rotina PI (escala o termo proporcional).
	- Exemplo: `SCG.10` define `GAIN` para 10.

- `SDO.NN` — Set Duty-cycle Open-loop
	- Formato: `SDO.` seguido por 1 a 2 dígitos decimais.
	- Ação: define a referência de duty-cycle em modo aberto (valor em %).
	- Exemplo: `SDO.75` define referência de duty para 75%.

- `SDS.NN` — Set Duty-cycle Step (open-loop)
	- Formato: `SDS.` seguido por 1 a 2 dígitos decimais.
	- Ação: define o incremento (`duty_step`) usado por `applyPWM_Ref()` para aproximar a referência gradualmente.
	- Exemplo: `SDS.05` define passo de 5%.

Observações:

- Os comandos são processados quando CR ou LF é recebido.
- Não envie caracteres extras (letras ou símbolos) após os dígitos esperados.

## Códigos de resposta

O parser retorna códigos internos que podem ser usados para controle de protocolo:

- `DEFAULT_RESPONSE` = 0  (comando indefinido / não reconhecido)
- `MA_RESPONSE` = 1       (resposta tipo MA — comandos de atuação)
- `MF_RESPONSE` = 3       (resposta tipo MF — comandos de malha fechada)

Esses códigos estão definidos em `EMCUA/uart/UART.h`.

## Observações finais

- Coloque as imagens `logo.png` e `datapool.jpg` na pasta `assets/` (crie a pasta se necessário) para que o README exiba as imagens corretamente.
- Para ajustar parâmetros (KP, KI, limites de DUTY, etc.) edite `EMCUA/motor/motor-controller.h`.

Bom trabalho! Se quiser, eu posso:

- copiar as imagens anexadas para `assets/` automaticamente,
- ou gerar um pequeno script para criar/organizar a pasta de assets.


