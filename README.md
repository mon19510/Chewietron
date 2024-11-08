# Chewietron
Este repositorio contiene el código, modelos 3D y la documentación para un robot cuadrúpedo de 12 DOF, controlado mediante cinemática inversa. El objetivo es implementar un sistema de control que permita al robot realizar movimientos precisos y estáticos mediante el control de cada una de sus articulaciones. Fue nombrado en honor a mi perro, Chewie.

<img src="https://github.com/mon19510/Chewietron/blob/main/imgs/Chewie.jpg" alt="Chewie original" width="300"/>

Este robot cuadrúpedo fue diseñado para estudiar y aplicar conceptos de cinemática inversa en un sistema físico. El robot cuenta con tres grados de libertad por pata, permitiéndole moverse de forma independiente en cada articulación. El control de cada servo se realiza utilizando un controlador PCA9685 conectado a un microcontrolador UM Tiny S3 (ESP32-S3), que calcula los ángulos de movimiento necesarios para cada pata con base en la posición deseada.



