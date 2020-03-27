# zips every .usda found in calling location to .usdz

for f in *.usda
do
    fname="${f%.*}"
    zipfile=${fname}.usdz
	echo "zipping $f to $zipfile"
    usdzip --arkitAsset $f $zipfile 
done
