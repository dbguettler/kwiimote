FROM ros:noetic
WORKDIR /
RUN apt-get update && apt-get install -y git libbluetooth-dev pip
RUN pip install python-kasa
RUN git clone https://github.com/wiiuse/wiiuse.git
RUN mkdir /wiiuse/build
WORKDIR /wiiuse/build
RUN cmake ..
RUN make install
WORKDIR /app
RUN rm -rf /wiiuse
RUN mkdir src
RUN . /opt/ros/noetic/setup.sh && catkin_make
COPY ./entry.sh /
RUN chmod +x /entry.sh
COPY ./kwii /app/src/kwii/
RUN . /opt/ros/noetic/setup.sh && catkin_make

ENTRYPOINT [ "/entry.sh" ]