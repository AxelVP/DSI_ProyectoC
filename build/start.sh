
#Iniciamos el AHRS
sudo ./ahrs > /dev/null 2>&1 &

#Iniciamos el mando
#sudo ./mando > /dev/null 2>&1 &

#Iniciamos el PID
sudo ./pid 0 > /dev/null 2>&1 &

#Iniciamos motores
# sudo ./motores

#Iniciamos el printer
# sudo ./printer