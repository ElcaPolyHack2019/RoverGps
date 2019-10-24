FROM ros:melodic-ros-base

WORKDIR /home/ros/gps

COPY ./dependencies-apt.txt ./

# install apt dependencies
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    $(awk -F: '/^[^#]/ { print $1 }' dependencies-apt.txt | uniq) \
  && rm -rf /var/lib/apt/lists/*

COPY . .

CMD [ "roslaunch", "./files/gps.launch" ]
