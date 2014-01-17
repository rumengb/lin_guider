rm -rf ./Debug ./Release/*.o ./Release/*.so ./Release/*.d ./.cproject ./.project ./libio_ftdi.so
find ./ -type d -iname .svn | xargs rm -rf
cd ./Release
make
rm *.d *.o
mv libio_ftdi.so ..

