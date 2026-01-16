@echo off

move Output\*.bin .\
move Output\*.hex .\

move *.bin Output\
move *.hex Output\
copy Checksum.exe Output\
cd Output
CheckSum.exe HP
copy *svnxxx_*.bin ..\Bin