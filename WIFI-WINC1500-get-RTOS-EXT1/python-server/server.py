from flask import Flask, render_template, request, jsonify
from flask_restful import Resource, Api

app = Flask(__name__)

global led
global button
global afec
global id
global time
button = 0
led = 0
afec = 0
id_placa = 0
time = 0

@app.route('/')
def control():
   return render_template('index.html')

@app.route('/status', methods = ['POST', 'GET','POST2'])
def status():
   global led,button,afec,id_placa,time
   if request.method == 'POST':
      print("entrou")
      status = request.form
      print(status)
      afec = status['AFEC']
      id_placa = status['ID']
      time = status['TIME']


      print(afec)
      print(id)
      print(time)

      return render_template("status.html", status = status)

   elif request.method == 'POST2':
      status = request.form
      print(status)
      button = status['BUT']
      print(button)

      return render_template("status.html", status = status)

   else:
      return jsonify({'id' : id_placa , 'but' : button,'afec' : afec,'time' : time}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)
