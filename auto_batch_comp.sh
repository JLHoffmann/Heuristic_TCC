#to understand how it works, think that it is on the epos folder... like trunk/auto_batch_comp.sh
((counter = 0+0))
if [ $# -eq 2 ]
then
	while [ $counter -lt $2 ];
	do
		((temp = $counter*4+3)) #makes all the sequential tests
		sed -i 's/static const unsigned int _channel_3 = '$old';/static const unsigned int _channel_3 = '$temp';/' ./include/pmu.h
		#echo temp
		make veryclean && make APPLICATION=$1 
		if [ $counter -lt 10 ]
		then
			cp ./img/$1.img ./../imgs/epos0$counter.img
		else
			cp ./img/$1.img ./../imgs/epos$counter.img
		fi
		((counter = $counter + 1))
		((old = $temp))
	done
	scp ./../imgs/*.img entropy_proxy:/var/ftpd/imgs/
else
	((old = 0+0))
	while [ $counter -lt 53 ]
	do
		((temp = $counter*4+3)) #makes all the sequential tests
		sed -i 's/static const unsigned int _channel_3 = '$old';/static const unsigned int _channel_3 = '$temp';/' ./include/pmu.h
		#echo temp
		make veryclean && make APPLICATION=$1 
		if [ $counter -lt 10 ]
		then
			cp ./img/$1.img ./../imgs/epos0$counter.img
		else
			cp ./img/$1.img ./../imgs/epos$counter.img
		fi
		((counter = $counter + 1))
		((old = $temp))
	done
	((temp = 214)) #makes the last test
	sed -i 's/static const unsigned int _channel_3 = '$old';/static const unsigned int _channel_3 = '$temp';/' ./include/pmu.h
	#echo temp
	make veryclean && make APPLICATION=$1 
	if [ $counter -lt 10 ]
	then
		cp ./img/$1.img ./../imgs/epos0$counter.img
	else
		cp ./img/$1.img ./../imgs/epos$counter.img
	fi
	scp ./../imgs/*.img entropy_proxy:/var/ftpd/imgs/
fi
