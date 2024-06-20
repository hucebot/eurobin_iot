FROM ros:humble

RUN apt-get update && apt-get install -y \
    x11-apps \
    xauth \
    xvfb \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-sync1 \
    libxcb-util1 \
    libxcb-xfixes0 \
    libxkbcommon-x11-0 \
    python3-pip \
    '~nros-humble-rqt*' 
    
   
RUN pip3 install Pyqt6


