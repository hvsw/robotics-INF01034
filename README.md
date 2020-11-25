# INSTALAÇÃO NO UBUNTU 18.04

Primeiramente, é preciso instalar a biblioteca ARIA e o simulador MobileSim

## 1. Instalação da ARIA

  A versão da biblioteca utilizada é a 2.7.2-mod (contendo modificações no código para funcionar nas versões mais novas do Ubuntu).

  Para instalá-la descompacte o arquivo `ARIA-2.7.2-mod.zip`, vá até a pasta `Aria-2.7.2-mod` e digite
```
make
```

  Depois, para a instalação da biblioteca ser feita no sistema (caminho /usr/local/Aria) é preciso digitar
```
sudo make install
```

  Também é preciso adicionar o caminho da biblioteca em `LD_LIBRARY_PATH`:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Aria/lib
sudo ldconfig  
```

  Para uma solução permanente, adicione o comando no `.bashrc`:
```
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Aria/lib' >> ~/.bashrc
```

## 2. Instalação do MobileSim

  Baixe o pacote `mobilesim_0.7.3+ubuntu12+gcc4.6_amd64.deb`

  Antes de instalá-lo, é preciso instalar algumas dependências do MobileSim:
```
sudo apt install fxfonts-100dpi
```

  Instale o MobileSim via:
```
dpkg -i mobilesim_0.7.3+ubuntu12+gcc4.6_amd64.deb
```

  Aí pra rodar é só abrir o programa MobileSim e depois selecionar um mapa (por exemplo: `3loops.map`)

## 3. Por fim, o framework possui duas depêndencias que devem ser instaladas
 - `Glut`: para gerenciamento de janelas, teclado, ...
 - `FreeImage`: para salvamento de capturas da tela
```
sudo apt-get install freeglut3-dev libfreeimage-dev 
```

# Como compilar e rodar o programa

Há duas formas de compilar o programa:

## Usando o Makefile

Vá até a pasta `phir2framework` e digite `make`. O programa vai ser compilado em uma pasta `../build-make` (que fica ao lado da pasta `phir2framework`) para rodar digite `../build-make/program`

## Usando o QtCreator

vá até a pasta `phir2framework` e abre o arquivo `PhiR2Framework.pro` no QtCreator. Compila e roda dentro da IDE (os arquivos gerados estarão em `../build-PhiR2Framework-Desktop-Debug` ou `../build-PhiR2Framework-Desktop-Release`)

OBS: antes de rodar o programa é preciso abrir o simulador MobileSIM (ou ligar o robô real, se for o caso). 
No simulador é preciso escolher um mapa. Use o mapa `3loops.map`, disponível no moodle.

# Controles do framework

-- modos de movimentação
`1`: controle manual simples (velocidades fixas)
`2`: controle manual velocidade linear (CIMA e BAIXO) e vel. angular (ESQ e DIR)
`3`: avoid obstacles
`4`: wall following

`CIMA`, `BAIXO`, `ESQ`, `DIR`: move o robô

`W`, `A`, `S`, `D` : move a camera

`+` e `-`: altera zoom da camera
`L`: fixa a camera novamente sobre o robô, ou deixa a camera fixa no ponto atual do mapa onde o robô está

`V` e `B` : altera visualização do mapa
`R`: altera visualização dos sensores (sonar cone -> sonar linha -> laser linha -> laser area -> somente robô) 
`G`: mostra valor associado a cada celula do mapa

ESC   : fecha programa
