# Daemonの設定方法

> [Raspberry Piでプログラムを自動起動する5種類の方法](https://qiita.com/karaage0703/items/ed18f318a1775b28eab4)

1. sshでラズパイに接続
2. 設定ファイルを作成。
	```bash
	sudo nano /etc/systemd/system/cansat.service
	```  
	`cansat.service`の中身は以下。
	```
	[Unit]
	Description = cansat daemon
	
	[Service]
	Type = simple
	Restart = no
	WorkingDirectory = {本番用コードのあるディレクトリ}
	ExecStart = {実行}
	StandardOutput = {標準出力先ファイル}
	StandardError = {エラー出力先ファイル}

    # 去年は以下のような感じ
    # WorkingDirectory = /home/pi/utat/cansat2022/main
	# ExecStart = /usr/bin/python /home/pi/utat/cansat2022/main/main.py
    # StandardOutput = file:/home/pi/utat/log/log.txt
    # StandardError = file:/home/pi/utat/log/error.txt
	
	[Install]
	WantedBy = multi-user.target
	```
3. 設定ファイルをロード
	```bash
	sudo systemctl load-daemon
	```  
	もともとのunit（この場合だと`cansat.service`）を編集して反映するには，
	```bash
	sudo systemctl daemon-reload
	```
	をする。  

デーモンの状態は、
```bash
sudo systemctl status cansat
```
で確認できる。  

また，いちいち電源切ったりしなくても，動作確認の際には，
```bash
sudo systemctl start cansat
```
と
```bash
sudo systemctl stop cansat
```
が便利。  