# Dockerfile containing software for Control C++ quizzes
FROM ubuntu:xenial

RUN apt-get update && apt-get install -y \
    build-essential \
    gcc \
    g++ \
    gfortran \
    cmake \
    pkg-config \
    unzip \
    git \
    wget \
    cppad \
    python-dev

RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.1.zip && unzip Ipopt-3.12.1 && rm Ipopt-3.12.1.zip
RUN bash install_ipopt.sh Ipopt-3.12.1

RUN rm -rf /var/lib/apt/lists/*
