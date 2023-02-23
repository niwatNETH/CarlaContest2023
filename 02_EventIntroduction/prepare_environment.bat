ren venv venv_TO_BE_REMOVED
rmdir /s /q venv_TO_BE_REMOVED

python -m venv ./venv
cd venv/Scripts
call activate
cd ../..
python -m pip install --upgrade pip
python -m pip install -r requirements.txt