p=$1
i=$2
d=$3
s=$4

./build/pid -p $p -i $i -d $d -s $s >record_${p}_${i}_${d}_${s}.csv
