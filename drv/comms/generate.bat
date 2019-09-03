set prev_dir=%cd%
cd %~dp0
protoc --nanopb_out=. protocol.proto
cd %prev_dir%