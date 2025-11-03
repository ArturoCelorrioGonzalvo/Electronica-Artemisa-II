Máquina de estados:

* Estado DEBUG :

 	-Leer sensores

 	-Guardar datos en la tarjeta SD

    -Enviar datos

 	-Resto de debugs

 	-No hay transiciones



* Estado TEST :

 	-Modo Bajo Consumo del ESP

 	-Recibiendo Datos del LoRa (Para transicionar al siguiente estado)

 	-Leer datos sensores y escribir por puerto Serie?



 	-Transición: Mandar instrucción por LoRa



* Estado ON\_PAD :

 	-Modo consumo normal

 	-Mandar datos por LoRa (usar un struct más pequeño que guardar datos en microSD)

 	-Leer sensores

 	-Guardar datos en la tarjeta SD

 	-En los siguientes estados se sigue manteniendo la lectura de sensores, el guardado

 	 de datos y el envío por LoRa



 	-Transición: Se detecta una aceleración de unos 3 a 5 Gs



* Estado SUBIENDO :

 	-Comprobar si hemos llegado al apogeo (guardando 5 valores de altura y mirando si disminuye)



 	-Transición: Detección del apogeo



* Estado PARACAIDAS\_1 :

 	-Desplegar paracaídas 1



 	-Transición: Altura disminuye hasta los 150m sobre la altura inicial.



* Estado PARACAÍDAS\_2 :

 	-Desplegar segundo paracaídas



 	-Transición: La altura ya no varía en el tiempo (cohete en el suelo)

 

* Estado ATERRIZAJE :

 	-Encender el buzzer

 	-Mandar estado por LoRa junto a coordenadas.

