export DRIBBLE=13
export KICKER=7
export CHARGE=26

pigs modes $DRIBBLE w
pigs modes $CHARGE w
pigs modes $KICKER w

function dribble () {
	pigs write $DRIBBLE $1
}

function charge () {
	pigs write $CHARGE $1
}

function kick () {
	pigs wvclr
	pigs wvag 0x80 0x00 $1 0x00 0x80 1000
	pigs wvcre
	pigs wvtx 0

	sleep 0.2
	pigs write $CHARGE 0
	sleep 0.2
	pigs write $CHARGE $2
}
