FROM ubuntu:18.04

RUN apt-get update \
    && apt-get install -y less \
    && apt-get install -y wget \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
# prepare for launching the installation of dependencies defined in install.sh
ADD install.sh install.sh
RUN sh ./install.sh && rm install.sh
# create user account, and create user home dir
RUN useradd -ms /bin/bash octave
# add specific packages needed for the execution of the code
RUN cd /home/octave \
    && wget -O control-3.2.0.tar.gz https://octave.sourceforge.io/download.php?package=control-3.2.0.tar.gz \
    && wget -O statistics-1.4.1.tar.gz https://octave.sourceforge.io/download.php?package=statistics-1.4.1.tar.gz \
    && wget -O io-2.4.12.tar.gz https://octave.sourceforge.io/download.php?package=io-2.4.12.tar.gz \
    && wget -O signal-1.4.1.tar.gz https://octave.sourceforge.io/download.php?package=signal-1.4.1.tar.gz \
    && wget -O mapping-1.2.1.tar.gz https://octave.sourceforge.io/download.php?package=mapping-1.2.1.tar.gz
ADD package_install.m /home/octave/package_install.m
RUN cd /home/octave \
    && /home/octave/package_install.m


# cp all code files into user home dir
RUN mkdir /home/octave/src
ADD src /home/octave/src/
ADD run_pi /home/octave/
# set the user as owner of the copied files.
RUN chown -R octave:octave /home/octave/

USER octave
WORKDIR /home/octave

# TODO check if this is required
# VOLUME ["/source"]
# ENTRYPOINT ["octave"] # When this is uncommented the container runs ocatave when accesing to it
