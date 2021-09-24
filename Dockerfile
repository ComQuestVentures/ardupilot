#FROM ubuntu:16.04
FROM bitnami/minideb:buster
WORKDIR /ardupilot

ARG DEBIAN_FRONTEND=noninteractive
RUN useradd -U -m ardupilot && \
    usermod -G users ardupilot

RUN apt-get update

RUN install_packages \
    gcc \
    g++ \
    lsb-release \
    sudo \
    software-properties-common && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

COPY Tools/environment_install/install-prereqs-ubuntu.sh /ardupilot/Tools/environment_install/
COPY Tools/completion /ardupilot/Tools/completion/

# Create non root user for pip
ENV USER=ardupilot

RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ardupilot
RUN chmod 0440 /etc/sudoers.d/ardupilot

RUN chown -R ardupilot:ardupilot /ardupilot

USER ardupilot

ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# add waf alias to ardupilot waf to .bashrc
RUN echo "alias waf=\"/ardupilot/waf\"" >> ~/.bashrc

# Check that local/bin are in PATH for pip --user installed package
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/.bashrc

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

# Cleanup
RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV CCACHE_MAXSIZE=1G
ENV PATH /usr/lib/ccache:/ardupilot/Tools:${PATH}
