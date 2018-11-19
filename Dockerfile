FROM ubuntu:18.04

COPY install-ubuntu.sh .
COPY install_ipopt.sh .
RUN apt-get update
RUN apt-get install sudo git libuv1-dev libssl-dev \
    gcc g++ cmake make wget unzip gfortran htop zlib1g-dev -y
RUN sh install-ubuntu.sh
RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && \
     unzip Ipopt-3.12.7.zip && \
     rm Ipopt-3.12.7.zip
RUN sh install_ipopt.sh Ipopt-3.12.7  
RUN apt-get install gcc-6 g++-6 cppad -y
# To Compile project remember use Gcc 6
# cmake -DCMAKE_C_COMPILER="/usr/bin/gcc-6" -DCMAKE_CXX_COMPILER="/usr/bin/g++-6" .. && make