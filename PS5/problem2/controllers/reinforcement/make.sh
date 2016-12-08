gcc -I. -I"/Applications/Webots/include/controller/c" -Wall -O3 -mmacosx-version-min=10.7 -DMACOS -MM youbot.c -MT build/release/youbot.o > build/youbot.d
gcc -c -Wall -O3 -mmacosx-version-min=10.7 -DMACOS -I. -I"/Applications/Webots/include/controller/c" youbot.c -o build/release/youbot.o
gcc -mmacosx-version-min=10.7 -o build/release/youbot build/release/*.o -L"/Applications/Webots/lib" -lController
cp build/release/youbot youbot > /dev/null 2>&1 || :
