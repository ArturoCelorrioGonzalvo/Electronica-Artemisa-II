Máquina de estados:

* Estado DEBUG : 

&nbsp;	-Leer sensores

&nbsp;	-Guardar datos en la tarjeta SD

&nbsp;	-Resto de debugs

&nbsp;	-No hay transiciones



* Estado TEST :

&nbsp;	-Modo Bajo Consumo del ESP

&nbsp;	-Recibiendo Datos del LoRa (Para transicionar al siguiente estado)

&nbsp;	-Leer datos sensores y escribir por puerto Serie?



&nbsp;	-Transición: Mandar instrucción por LoRa



* Estado ON\_PAD : 

&nbsp;	-Modo consumo normal

&nbsp;	-Mandar datos por LoRa (usar un struct más pequeño que guardar datos en microSD)

&nbsp;	-Leer sensores

&nbsp;	-Guardar datos en la tarjeta SD

&nbsp;	-En los siguientes estados se sigue manteniendo la lectura de sensores, el guardado 

&nbsp;	 de datos y el envío por LoRa



&nbsp;	-Transición: Se detecta una aceleración de unos 3 a 5 Gs



* Estado SUBIENDO :

&nbsp;	-Comprobar si hemos llegado al apogeo (guardando 5 valores de altura y mirando si 	 disminuye)



&nbsp;	-Transición: Detección del apogeo



* Estado PARACAIDAS\_1 :

&nbsp;	-Desplegar paracaídas 1



&nbsp;	-Transición: Altura disminuye hasta los 150m sobre la altura inicial.



* Estado PARACAÍDAS\_2 :

&nbsp;	-Desplegar segundo paracaídas



&nbsp;	-Transición: La altura ya no varía en el tiempo (cohete en el suelo)

&nbsp;	

* Estado ATERRIZAJE :

&nbsp;	-Encender el buzzer

&nbsp;	-Mandar estado por LoRa junto a coordenadas.





