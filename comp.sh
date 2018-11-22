make veryclean && make APPLICATION=$1 && scp ./img/$1.img entropy_proxy:/var/ftpd/epos.img
