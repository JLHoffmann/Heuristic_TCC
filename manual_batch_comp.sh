((counter = 0+0))
if [ $# -eq 2 ]
then
	while [ $counter -lt $2 ];
	do
		echo "foid"
		make veryclean && make APPLICATION=$1 && scp ./img/$1.img entropy_proxy:/var/ftpd/imgs/epos$counter.img
		((counter = $counter +1))
	done
else
	while true
	do
		make veryclean && make APPLICATION=$1 && scp ./img/$1.img entropy_proxy:/var/ftpd/imgs/epos$counter.img
		((counter = $counter + 1))
	done
fi
