# KFusion
Las nubes de puntos se guardan en "C:\Users\Usuario\3D Objects\PruebaFusion" y las poses en "filePoses.txt" en la misma carpeta del proyecto de prueba. Se ejecuta con la condición de que pasen 3 segundos pero se desea que se haga cuando la cámara se traslade o rote una distancia o ángulo determinado. Con los datos obtenidos de un escaneo (rotación 360º en una habitación por ejemplo) se importan a cloudCompare y se les aplica la transformación de la última pose de la cámara en el volumen anterior.

En ProyectoConHilos2 (da fallos), se pretende crear un hilo de guardado en la clase KinectFusionSaver. El hilo de procesado calcula la malla (Fusion Processor) . Y el programa de la ventana principal (KFusionExplorer) envía la malla al hilo de guardado.
