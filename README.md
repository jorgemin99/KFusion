# KFusion
DlgProc es un método que se ejecuta continuamente en KinectFusionExplorer.cpp. Se aprovecha para agregar una condición que ejecute un salvado de la nube de puntos, la pose worldToCamera y resetee el volumen. (linea 187)
Las nubes de puntos se guardan en "C:\Users\Usuario\3D Objects\PruebaFusion" y las poses en "filePoses.txt" en la misma carpeta del proyecto de prueba. Se ejecuta con la condición de que pasen 3 segundos pero se desea que se haga cuando la cámara se traslade o rote una distancia o ángulo determinado. Con los datos obtenidos de un escaneo (rotación 360º en una habitación por ejemplo) se importan a cloudCompare y se les aplica la transformación de la última pose de la cámara en el volumen anterior.
