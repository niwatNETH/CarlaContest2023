@echo off
title Exeute agent

cd venv/Scripts
call activate
cd ../..
python runner.py