FROM debian:latest
WORKDIR /
RUN apt-get update && apt-get install -y git make gcc cmake g++ libbluetooth-dev
RUN git clone https://github.com/wiiuse/wiiuse.git
RUN mkdir /wiiuse/build
WORKDIR /wiiuse/build
RUN cmake ..
RUN make install
WORKDIR /
RUN apt-get install -y pip
RUN pip install python-kasa
WORKDIR /app
COPY ./kwii-input.c ./Makefile ./
RUN make
ENTRYPOINT [ "/app/kwii-input" ]
# ENTRYPOINT [ "/usr/local/bin/kasa" ]