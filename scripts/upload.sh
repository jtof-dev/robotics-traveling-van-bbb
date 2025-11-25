#!/bin/bash

if [ $(id -u) -ne 0] 
then echo Please run this script as root or using sudo!
	exit
fi

if [ $1 -z ] then
	echo "Please put in the drive"
	exit

fi

mkdir RPI_PICO_UPLOAD_FOLDER
mount 
