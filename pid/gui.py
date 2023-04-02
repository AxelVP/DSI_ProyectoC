#sudo apt-get install python3-tk
#python3 gui.py
#Actualiza los valores de un fichero de los que el .c del pid tomara el roll pitch y yaw en tiempo de ejecucion

import sys
from tkinter import *

def hacer_click():
 try:
  _valorKp = float(entrada_textoKp.get())
  _valorKd = float(entrada_textoKd.get())
  _valorKi = float(entrada_textoKi.get())
  
  etiquetaKp.config(text="Nuevo valor de Kp: " + str(_valorKp))
  etiquetaKd.config(text="Nuevo valor de Kd: " + str(_valorKd))
  etiquetaKi.config(text="Nuevo valor de Ki: " + str(_valorKi))

  actualizar_fichero(_valorKp, _valorKd, _valorKi)
 except ValueError:
  etiquetaKp.config(text="Introduce un numero!")

#Leer el fichero con los valores
def leer_fichero():
  f = open("pid.txt","r")
  array_valores = f.read().split(" ")
  f.close()
  return array_valores

#Actualizar los valores del fichero
def actualizar_fichero(Kp, Kd, Ki):
  f = open("pid.txt","w")
  array_valores = str(Kp) + " " + str(Kd) + " " + str(Ki) + " "
  f.write(array_valores)
  f.close()

app = Tk()
app.title("PID GUI")

#Ventana Principal
vp = Frame(app)
vp.grid(column=0, row=0, padx=(50,50), pady=(10,10))
vp.columnconfigure(0, weight=1)
vp.rowconfigure(0, weight=1)

#Leer los valores anteriores
array_valores = leer_fichero()


#PARTE Kp
etiquetaKp = Label(vp, text="Valor Kp: " + array_valores[0])
etiquetaKp.grid(column=2, row=1, sticky=(W,E))

valorKp = ""
entrada_textoKp = Entry(vp, width=10, textvariable=valorKp)
entrada_textoKp.insert(0,array_valores[0])
entrada_textoKp.grid(column=1, row=1)

#PARTE Kd
etiquetaKd = Label(vp, text="Valor Kd: " + array_valores[1])
etiquetaKd.grid(column=2, row=2, sticky=(W,E))

valorKd = ""
entrada_textoKd = Entry(vp, width=10, textvariable=valorKd)
entrada_textoKd.insert(0,array_valores[1])
entrada_textoKd.grid(column=1, row=2)

#PARTE Ki
etiquetaKi = Label(vp, text="Valor Ki: " + array_valores[2])
etiquetaKi.grid(column=2, row=3, sticky=(W,E))

valorKi = ""
entrada_textoKi = Entry(vp, width=10, textvariable=valorKi)
entrada_textoKi.insert(0,array_valores[2])
entrada_textoKi.grid(column=1, row=3)


#BOTON ACTUALIZAR TODO
botonKp = Button(vp, text="Actualizar valores", command=hacer_click)
botonKp.grid(column=1, row=4)

app.mainloop()