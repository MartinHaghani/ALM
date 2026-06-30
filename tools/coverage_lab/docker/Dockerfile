FROM ubuntu:22.04

ARG FIELDS2COVER_REF=v2.0.0
ARG DEBIAN_FRONTEND=noninteractive

LABEL org.opencontainers.image.title="open_mower_coverage_lab" \
      org.opencontainers.image.description="Laptop-only Fields2Cover planner lab for OpenMower maps" \
      org.opencontainers.image.version="fields2cover-v2.0.0"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    doxygen \
    g++ \
    git \
    gnuplot \
    lcov \
    libboost-dev \
    libeigen3-dev \
    libgdal-dev \
    libgeos-dev \
    libgtest-dev \
    libpython3-dev \
    libtbb-dev \
    libtinyxml2-dev \
    nlohmann-json3-dev \
    pkg-config \
    python3 \
    python3-matplotlib \
    python3-pip \
    python3-tk \
    python3-yaml \
    swig \
  && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch "${FIELDS2COVER_REF}" https://github.com/Fields2Cover/Fields2Cover.git /tmp/Fields2Cover \
  && cmake -S /tmp/Fields2Cover -B /tmp/Fields2Cover/build \
      -DBUILD_PYTHON=ON \
      -DBUILD_TUTORIALS=OFF \
      -DBUILD_TESTING=OFF \
      -DBUILD_DOC=OFF \
      -DCMAKE_BUILD_TYPE=Release \
  && cmake --build /tmp/Fields2Cover/build --target install --parallel "$(nproc)" \
  && ldconfig \
  && python3 -c "import fields2cover" \
  && rm -rf /tmp/Fields2Cover

# Shapely (GEOS Python bindings) is used by tools/coverage_lab/lab_geometry.py for
# footprint-aware headland erosion (P0) and obstacle-aware cell decomposition (P1).
# libgeos-dev is already installed above for Fields2Cover, so shapely picks it up.
RUN pip3 install --no-cache-dir "shapely>=2.0,<3" \
  && python3 -c "import shapely; print('shapely', shapely.__version__)"

# Pillow powers the lawn_preview.png output in render_run() -- raster
# compositing of cutting disk + footprint + wheel tracks in pose order.
RUN pip3 install --no-cache-dir "Pillow>=10,<12" \
  && python3 -c "import PIL; print('Pillow', PIL.__version__)"

WORKDIR /workspace
ENTRYPOINT ["python3", "/workspace/tools/coverage_lab/coverage_lab.py"]
