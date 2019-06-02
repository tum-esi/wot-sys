if [ $1 = "temp" ]
then
    sudo gatttool -b $2 --char-write-req -a 0x24 -n 01; sleep 1;  sudo gatttool -b $2 --char-read -a 0x21; sleep 1; sudo gatttool -b $2 --char-write-req -a 0x24 -n 00 # temp
elif [ $1 = "humidity" ]
then   
    sudo gatttool -b $2 --char-write-req -a 0x2C -n 01; sleep 1;  sudo gatttool -b $2 --char-read -a 0x29; sleep 1; sudo gatttool -b $2 --char-write-req -a 0x2C -n 00  #hum
elif [ $1 = "barometer" ]
then
    sudo gatttool -b $2 --char-write-req -a 0x34 -n 01; sleep 1;  sudo gatttool -b $2 --char-read -a 0x31;  sleep 1; sudo gatttool -b $2 --char-write-req -a 0x34 -n 00 #baro
elif [ $1 = "gyro_acc_mag" ]
then
    sudo gatttool -b $2 --char-write-req -a 0x3C -n FFFF; sudo gatttool -b $2 --char-read -a 0x39
 #gyro,acc
elif [ $1 = "lux" ] 
then
    sudo gatttool -b $2 --char-write-req -a 0x44 -n 01; sleep 1;  sudo gatttool -b $2 --char-read -a 0x41; sleep 1;  sudo gatttool -b $2 --char-write-req -a 0x44 -n 00 #luxo
else
    echo "no such sensor exists"
fi
