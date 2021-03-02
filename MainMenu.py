"""
Menú principal y vistas referentes a los datos tomados por la estación de datos biométricos.

Autores:
Erick Alberto Bustos Cruz
Daniela Avila Luna
"""


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from os import system, name
import sys
import mysql.connector
from statistics import mean 


def conexion_sql():
    conexion=mysql.connector.connect(host="localhost",
                                  user="-",
                                  passwd="-",
                                  database="-")
    return conexion


def clear():
    if name == 'nt':
        _ = system('cls')
        
def opcion0(cursor,conexion):
    """ 
    Opción para tomar las mediciones del día. Se preguntan apellidos, nombre, edad, sexo 
    y una serie de datos cualitativos que pueden ser indicadores del COVID-19. Se busca a la persona en 
    la base de datos y se crea un nuevo registro de sus mediciones con la fecha actual. Después de 
    seleccionar esta opción la persona debe utilizar la estación de toma de datos
    biométricos para mandar los datos cuantitativos.
    """
    try:
        clear()
        print("\nMIS MEDICIONES DE HOY")
        print("\nIngrese sus datos:")
        apellidos = input("Ingresa tus apellidos: ")
        nombres = input("Ingresa tu nombre(s): ")

        edad = input("Ingresa tu edad: ")
        sexo = input("Ingresa tu sexo [M para masculino, F para femenino, O para otro]: ")
        CP = input("Ingrese el código postal del lugar donde se encuentra: ")
        print("\nEn la última semana ...")
        congestion_nasal = input("¿Ha tenido congestion nasal? Ingrese si/no: ")
        dolor_cabeza = input("¿Ha tenido dolor de cabeza? Ingrese si/no: ")
        tos_seca = input("¿Ha tenido tos seca? Ingrese si/no: ")
        cansancio = input("¿Ha sentido cansancio fuera de lo normal? Ingrese si/no: ")
        if(sexo.lower() in ['m','hombre']):
            sexo = 'masculino'
        elif(sexo.lower() in ['f','mujer']):
            sexo = 'femenino'
        elif(sexo.lower() in ['o','otro','0']):
            sexo = 'otro'
        query = f'SELECT ID_usuario FROM persona WHERE nombres = "{nombres}" AND apellidos = "{apellidos}" AND sexo = "{sexo}" AND TIMESTAMPDIFF(YEAR, persona.fecha_nacimiento,CURDATE()) = {edad}'
        cursor.execute(query)
        bd = pd.DataFrame(columns = ["ID_usuario"])
        
        for fila in cursor:
            bd = bd.append({"ID_usuario":fila[0]}, ignore_index=True)
        
        ID_usuario = bd.iat[0,0]
        print(ID_usuario)
        conexion.close()
        
        conexion2 = conexion_sql()
        cursor2 = conexion2.cursor()
        
        query2 = f'INSERT INTO datos_biometricos (ID_usuario,codigo_postal,congestion_nasal,dolor_cabeza, tos_seca,cansancio) VALUES ({ID_usuario},"{CP}","{congestion_nasal}","{dolor_cabeza}","{tos_seca}","{cansancio}")'
        cursor2.execute(query2)
        conexion2.commit()
        
        
        print("\nMUCHAS GRACIAS, POR FAVOR PROCEDA A TOMAR SUS MEDIDAS BIOMÉTRICAS\n")
        
    except:
        print("\nERROR: No se pudo realizar la accion, por favor revise que hayan introducido los datos correctos.\n")
    


def opcion1(cursor, conexion): 
    """
    Se realiza un promedio semanal de los indicadores cuantitativos y cualitativos para un cierto usuario.
    En caso de que los datos impliquen una posible infección por COVID-19, se emite una alerta.
    """
    clear()
    print("\nMI DIAGNÓSTICO")
    #promedio de una semana
    nombre = input("Ingresa tu nombre: ")
    apellido = input("Ingresa tus apellidos: ")
    query = f'SELECT week(db.fecha) as "Semana", p.ID_usuario as "ID Usuario", p.nombres as "Nombre",p.apellidos as "Apellido", AVG(db.saturacion_oxigeno) as "Saturacion Oxigeno", AVG(db.ritmo_cardiaco) as "Ritmo cardiaco", AVG(db.temperatura) as "Temperatura", COUNT( CASE WHEN db.congestion_nasal = "si" THEN 1 END ) AS "Veces con congestion nasal", COUNT( CASE WHEN db.dolor_cabeza = "si" THEN 1 END ) AS "Veces con dolor de cabeza", COUNT( CASE WHEN db.tos_seca = "si" THEN 1 END ) AS "Veces con tos seca", COUNT( CASE WHEN db.cansancio = "si" THEN 1 END ) AS "Veces con cansancio" FROM datos_biometricos db, persona p WHERE db.ID_usuario = p.ID_usuario AND DATEDIFF(NOW(),db.fecha) < 15 AND p.nombres LIKE "{nombre}" AND p.apellidos LIKE "{apellido}" GROUP by week(db.fecha), p.ID_usuario'
    cursor.execute(query)
    bd = pd.DataFrame(columns = ["Semana", "ID_Usuario", "Nombre", "Apellido", "Promedio Saturacion Oxigeno", "Promedio Ritmo Cardiaco", "Promedio Temperatura", "Veces con congestion nasal", "Veces con dolor de cabeza", "Veces con tos seca", "Veces con cansancio"])

    for fila in cursor:
        bd = bd.append({"Semana": fila[0], "ID_Usuario": fila[1], "Nombre": fila[2], "Apellido": fila[3], "Promedio Saturacion Oxigeno": fila[4], "Promedio Ritmo Cardiaco": fila[5], "Promedio Temperatura": fila[6], "Veces con congestion nasal": fila[7], "Veces con dolor de cabeza": fila[8], "Veces con tos seca": fila[9], "Veces con cansancio": fila[10]}, ignore_index=True)

    conexion.close()
    print(bd)

    semanas = ["Semana 1","Semana 2"]
    promOx=bd["Promedio Saturacion Oxigeno"].tolist()
    promRpm = bd["Promedio Ritmo Cardiaco"].tolist()
    promTemp = bd["Promedio Temperatura"].tolist()
    congestion = bd["Veces con congestion nasal"].tolist()
    cabeza = bd["Veces con dolor de cabeza"].tolist()
    cansancio = bd["Veces con cansancio"].tolist()
    tosseca = bd["Veces con tos seca"].tolist()
    x = np.arange(len(semanas))
    width = 0.35 
    fig, ax = plt.subplots()
    rects2 = ax.bar(x -0.5 + width/3 , promRpm, width, label='RPM [bpm]')
    rects1 = ax.bar(x - width/3, promOx, width, label='Oxigenacion [%SpO2]')
    rects3 = ax.bar(x + width/3, promTemp, width, label='Temperatura [C°]')

    condTemp = mean(promTemp) > 37 or mean(promTemp) < 36
    condOx = mean(promOx) < 95
    condRpm = mean(promRpm) < 60
    sintomas = sum(congestion) > 4 or sum(cabeza) > 4 or sum(cansancio) > 4 or sum(tosseca) > 4


    #if(pulso y temp alarmantes) poner mensaje "Acuda al médico"
    if (condTemp and condOx and sintomas):
        print("\nALERTA 1: Sus datos de las últimas dos semanas indican que usted\npuede haber contraido el virus COVID-19. POR FAVOR CONSULTE A SU\nMÉDICO A LA BREVEDAD Y ABSTENGASE DEL CONTACTO CON OTRAS PERSONAS.\n")

        if( mean(promTemp) > 37 and condRpm):
            print("\nALERTA 2: su pulso promedio es muy bajo, ello suele ser\n un indicador de riesgo en pacientes con COVID-19, ACUDA A UN HOSPITAL PARA SU REVISIÓN.")
    else:
        print("\nSus datos indican que usted no presenta ningún síntoma\nnotorio de COVID-19. Sin embargo, recuerde seguir tomando sus precauciones.\n")
        

    ax.set_ylabel("Promedios")
    ax.set_xticks(x)
    ax.set_title("Estadísticas de 2 semanas")
    ax.set_xticklabels(semanas)
    ax.legend(loc=8, prop={'size': 6})

    rectas = [rects1, rects2, rects3]

    for x in rectas:
        for rect in x:
            height = rect.get_height()
            ax.annotate('{}'.format(round(height,2)),
                    xy=(rect.get_x() + rect.get_width() / 2, height-2),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')

    plt.show()
    
    #Oxigenación 95-100 NORMAL, menor a 95 WARNING
    #Temperatura 36.5°C to 37°C NORMAL, diferente WARNING
    #Ritmo cardiaco 60 a 100 NORMAL
    
    input("Ingrese cualquier letra para regresar: ")


def opcion2(cursor, conexion): 
    """
    Esta vista genera una gráfica de barras de todos los códigos postales con al menos un posible
    caso de COVID-19.
    """
    clear()
    print("\nZONAS CON MÁS SOSPECHOSOS")
    cursor.execute("SELECT datos_biometricos.codigo_postal as 'Zona', COUNT(DISTINCT persona.ID_usuario) as 'Número de Posibles Infectados' FROM persona, datos_biometricos WHERE datos_biometricos.ID_usuario = persona.ID_usuario AND DATEDIFF(NOW(),datos_biometricos.fecha) < 8 AND (datos_biometricos.temperatura > 37 OR datos_biometricos.temperatura < 37) AND datos_biometricos.saturacion_oxigeno < 95 AND ((datos_biometricos.congestion_nasal = 'si') OR (datos_biometricos.dolor_cabeza = 'si') OR (datos_biometricos.tos_seca = 'si') OR (datos_biometricos.cansancio = 'si')) GROUP BY datos_biometricos.codigo_postal ")
    bd= pd.DataFrame(columns = ["Zona","Numero de Posibles Infectados"])
    
    for fila in cursor:
        bd = bd.append({ 'Zona': fila[0], 'Numero Posibles Infectados':fila[1]},
                       ignore_index=True)
    conexion.close()

    bd.style
    bd.plot(kind = 'bar', x = 'Zona', y = 'Numero Posibles Infectados')
    plt.show()
    input("Ingrese cualquier letra para regresar: ")
    
    
def opcion3(cursor, conexion): 
    """
    Esta vista se encarga de proporcionar los nombres completos de las personas que tengan
    alguna condición que los categorice como de alto riesgo. Se muestran los nombres en una
    tabla o data frame.
    """
    clear()
    print("\nPERSONAS REGISTRADAS DE ALTO RIESGO")
    cursor.execute('select p.ID_usuario, p.nombres as "Nombre", p.apellidos as "Apellidos" from persona p, persona_condicion pc, condicion_de_salud cs where (p.ID_usuario = pc.ID_usuario AND pc.ID_condicion = cs.ID_condicion AND cs.riesgo_covid = "alto") OR TIMESTAMPDIFF(YEAR, p.fecha_nacimiento, CURDATE()) > 59 GROUP BY p.ID_usuario, p.nombres,  p.apellidos')
    bd = pd.DataFrame(columns = ["ID_Usuario","Nombre", "Apellido"])

    for fila in cursor:
        bd = bd.append({"ID_Usuario" : fila[0], "Nombre": fila[1], "Apellido": fila[2]}, ignore_index=True)
    conexion.close()
    print(bd)
    input("Ingrese cualquier letra para regresar: ")


def opcion4(cursor, conexion):
    """
    Esta vista proporciona una tabla o data frame que muestra todos los códigos postales en
    donde se registraron los datos biométricos de una persona.
    """
    clear()
    print("\nRASTREO")
    apellido = input("Ingrese los apellidos: ")
    nombre = input("Ingrese el nombre: ")
    query = f'SELECT db.codigo_postal as "Código Postal", p.nombres as "Nombre", p.apellidos as "Apellido" from datos_biometricos db, persona p where db.ID_usuario = p.ID_usuario and p.nombres like "{nombre}" and p.apellidos like "{apellido}" GROUP BY db.codigo_postal, p.nombres, p.apellidos'
    cursor.execute(query)
    bd = pd.DataFrame(columns = ["Código Postal","Nombre","Apellido"])

    for fila in cursor:
        bd = bd.append({"Código Postal": fila[0], "Nombre": fila[1], "Apellido": fila[2]}, ignore_index=True)
    conexion.close()
    print(bd)    
    input("Ingrese cualquier letra para regresar: ")


def opcion5(cursor, conexion):
    """
    Esta vista permite que el usuario introduzca su código postal y pueda saber el número de
    posibles casos de COVID-19 cuyos registros se realizaron en dicha zona.
    """
    clear()
    print("\nPOSIBLES CASOS EN MI ZONA")
    codigo_postal = input("Ingresa tu código postal: ")
    
    cursor.execute(f"SELECT datos_biometricos.codigo_postal as 'Zona', COUNT(DISTINCT persona.ID_usuario) as 'Número de Posibles Infectados' FROM persona, datos_biometricos WHERE datos_biometricos.ID_usuario = persona.ID_usuario AND datos_biometricos.codigo_postal = '{codigo_postal}' AND DATEDIFF(NOW(),datos_biometricos.fecha) < 8 AND (datos_biometricos.temperatura > 37 OR datos_biometricos.temperatura < 37) AND datos_biometricos.saturacion_oxigeno < 95 AND ((datos_biometricos.congestion_nasal = 'si') OR (datos_biometricos.dolor_cabeza = 'si') OR (datos_biometricos.tos_seca = 'si') OR (datos_biometricos.cansancio = 'si')) GROUP BY datos_biometricos.codigo_postal ")

    bd= pd.DataFrame(columns = ["Zona","Numero de Posibles Infectados"])

    for fila in cursor:
        bd = bd.append({ 'Zona': fila[0], 'Numero de Posibles Infectados':fila[1]},
                       ignore_index=True)
    conexion.close()
    bd.style 
    print(bd)
    input("Ingrese cualquier letra para regresar: ")


def opcion6(cursor, conexion): 
    """
    Esta vista regresa una tabla con los ID, nombres, apellidos, los promedios de temperatura y
    los promedios de oxigenación de las personas que han presentado indicios de haber contraído
    el virus en los últimos 7 días.
    """
    clear()
    print("\nPOSIBLES CASOS DE COVID-19")
    cursor.execute("SELECT persona.ID_usuario, persona.nombres, persona.apellidos, ROUND(AVG(datos_biometricos.temperatura),2), ROUND(AVG(datos_biometricos.saturacion_oxigeno),2) FROM persona, datos_biometricos WHERE datos_biometricos.ID_usuario = persona.ID_usuario AND DATEDIFF(NOW(),datos_biometricos.fecha) < 8 GROUP BY datos_biometricos.ID_usuario HAVING (AVG(datos_biometricos.temperatura) > 37 OR AVG(datos_biometricos.temperatura) < 36) AND AVG(datos_biometricos.saturacion_oxigeno) < 95 AND (COUNT(datos_biometricos.congestion_nasal = 'si') > 4 OR COUNT(datos_biometricos.dolor_cabeza = 'si') > 4 OR COUNT(datos_biometricos.tos_seca = 'si') > 4 OR COUNT(datos_biometricos.cansancio = 'si') > 4)")
    bd= pd.DataFrame(columns = ["ID Usuario","Nombre(s)","Apellido(s)","Promedio Temperatura [°C]","Promedio Saturación Oxígeno [%SpO2]"])

    for fila in cursor:
        bd = bd.append({ 'ID Usuario': fila[0], 'Nombre(s)':fila[1], 'Apellido(s)':fila[2],'Promedio Temperatura [°C]':fila[3], 'Promedio Saturación Oxígeno [%SpO2]':fila[4]},
                       ignore_index=True)
    conexion.close()
    bd.style 
    print(bd)
    input("Ingrese cualquier letra para regresar: ")
    


def main():
    while True:
        clear()
        conexion = conexion_sql()
        cursor = conexion.cursor()
        print("Reto de IoT\n\n0)Mis mediciones de hoy\n1)Mi diagnóstico\n2)Zonas con más sospechosos\n3)Personas registradas de alto riesgo\n4)Rastreo\n5)Posibles casos en mi zona\n6)Posibles casos de COVID-19\n7)SALIR")
        opcion = int(input("Seleccione una opción: "))
        if (opcion == 0):
            opcion0(cursor,conexion)
        elif(opcion == 1):
            opcion1(cursor, conexion)
        elif(opcion == 2):
            opcion2(cursor, conexion)
        elif(opcion == 3):
            opcion3(cursor, conexion)
        elif(opcion == 4):
            opcion4(cursor, conexion)
        elif(opcion == 5):
            opcion5(cursor, conexion)
        elif(opcion == 6):
            opcion6(cursor, conexion)
        elif(opcion == 7):
            conexion.close()
            sys.exit()
        else:
            print("No ingresó una opción correcta.")       

main()