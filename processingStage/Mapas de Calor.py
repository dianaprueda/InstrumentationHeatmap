# -*- coding: utf-8 -*-
"""Mapas de calor.ipynb

Automatically generated by Colab.

Original file is located at
    https://colab.research.google.com/drive/13q4t3-_9iiSybzlGI-gWNqE25V3yf97f
"""

!ls

# Commented out IPython magic to ensure Python compatibility.
# %cd /content/drive

pip install matplotlib pandas

from google.colab import drive
drive.mount('/content/drive')

import pandas as pd
import matplotlib.pyplot as plt

# Lee el archivo CSV y carga los datos en un DataFrame
data = pd.read_csv('/content/drive/MyDrive/Colab Notebooks/Paper Margarita/ReportededatosSaldana.csv', decimal=',',sep=';', encoding = 'unicode_escape')
data

pip install pandas matplotlib pykrige

# Extrae las columnas de interés
longitudes = data['LONGITUD']
latitudes = data['LATITUD']
temperaturas = data['Temperatura Â°C']
humedad = data['Humedad de Suelo %']
no3mV = data['NO3 (mv)']
potasio=data['Potasio (mv)']
potasiomM = data['LOG (K+ (M))']
no3mM=data['LOG (NO3- (M))']

# Commented out IPython magic to ensure Python compatibility.
# %cd /content/drive/MyDrive/Colab Notebooks/Paper Margarita/Imagenes con Datos revisados

from re import U
# Crea una malla de puntos para la interpolación ESTE ES EL DE LA IMAGEN
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

x = longitudes.values
y = latitudes.values
z = temperaturas.values
w = humedad.values
u = no3mV.values
v = potasio.values
r = no3mM.values
s = potasiomM.values

i=z<34
z=z[i]
x=x[i]
y=y[i]
w=w[i]
u=u[i]
v=v[i]
r=r[i]
s=s[i]


plt.scatter(x, y, c=z, cmap='GnBu')
plt.colorbar(label='Temperature, ºC')  # Añade una barra de color para la escala de temperaturas
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Temperature measurement')

# Muestra el mapa de calor

plt.savefig('Figure 1 Temperature.png', dpi=600)   #Cambie dpi a 600 Jun2024
plt.show()
# Crea el mapa de calor




plt.scatter(x, y, c=w, cmap='GnBu')
plt.colorbar(label='Soil Moisture, %')  # Añade una barra de color para la escala de temperaturas
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Soil moisture measurement')
#plt.savefig('Figura 1b Humedad.png', dpi=600) #Cambie dpi a 600 Jun2024
# Muestra el mapa de calor
plt.savefig('Figure 1b Moisture.png', dpi=600)    #Cambie dpi a 600 Jun2024
plt.show()



plt.scatter(x, y, c=u, cmap='GnBu')
plt.colorbar(label='NO3- (mV)')  # Añade una barra de color para la escala de temperaturas
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Nitrogen ions measurement')
#plt.savefig('Figura 1b Humedad.png', dpi=600)  #Cambie dpi a 600 Jun2024
# Muestra el mapa de calor
plt.savefig('Figure 1c ionesNi.png', dpi=600)   #Cambie dpi a 600 Jun2024
plt.show()

plt.scatter(x, y, c=v, cmap='GnBu')
plt.colorbar(label='Potassium (mV)')  # Añade una barra de color para la escala de temperaturas
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Potassium ions measurement')
#plt.savefig('Figura 1b Humedad.png', dpi=600)   #Cambie dpi a 600 Jun2024
# Muestra el mapa de calor
plt.savefig('Figure 1c ionesK.png', dpi=600)   #Cambie dpi a 600 Jun2024
plt.show()


plt.scatter(x, y, c=r, cmap='GnBu')
plt.colorbar(label='NO3- (log M)')  # Añade una barra de color para la escala de temperaturas
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Nitrogen ions ')
#plt.savefig('Figura 1b Humedad.png', dpi=600)  #Cambie dpi a 600 Jun2024
# Muestra el mapa de calor
plt.savefig('Figure 1d ionesNimM.png', dpi=600)   #Cambie dpi a 600 Jun2024
plt.show()

plt.scatter(x, y, c=s, cmap='GnBu')
plt.colorbar(label='Potassium (log M)')  # Añade una barra de color para la escala de temperaturas
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Potassium ions ')
#plt.savefig('Figura 1b Humedad.png', dpi=600)   #Cambie dpi a 600 Jun2024
# Muestra el mapa de calor
plt.savefig('Figure 1e ionesKmM.png', dpi=600)   #Cambie dpi a 600 Jun2024
plt.show()

# Graficar los datos originales y el orden en el que estan medidos
plt.scatter(x, y, facecolors='purple', edgecolors='purple')
#for i, (xi, yi) in enumerate(zip(x, y)):
#    plt.annotate(str(i), (xi, yi), textcoords="offset points", xytext=(0, 5), ha='center')

plt.xlabel('Longitude')
plt.ylabel('Latitude')
#plt.title('Data point locations')
plt.savefig('Figure 1 Datalocation.png', dpi=600)   #Cambie dpi a 600 Jun2024

plt.show()

#este si

# aqui hice lo de la concatenacion del grid. -funciona
# LUEGO: hacer que pueda restar z con zgrid para la posicion en x, y)

import pandas as pd
import matplotlib.pyplot as plt
from pykrige.ok import OrdinaryKriging
import numpy as np
from scipy.interpolate import griddata
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error

# Define los límites y la resolución de la malla de puntos
x_min, x_max = x.min(), x.max()
y_min, y_max = y.min(), y.max()
x_min=x_min-0.00002
x_max=x_max+0.00002
y_min=y_min-0.00001
y_max=y_max+0.00001

grid_resolution = 1000  # Ajusta la resolución según tus necesidades

x_grid = np.linspace(x_min, x_max, grid_resolution)
y_grid = np.linspace(y_min, y_max, grid_resolution)


x_grid_mesh, y_grid_mesh = np.meshgrid(x_grid, y_grid)

# Combinación de las coordenadas x_grid e y_grid en una malla 2D
xiyi_mesh = np.column_stack((x_grid_mesh.ravel(), y_grid_mesh.ravel()))

coordinates=xiyi_mesh

# Realiza la interpolación Kriging
OK = OrdinaryKriging(x, y, z, variogram_model='linear', verbose=False)
z_grid, ss = OK.execute('grid', x_grid, y_grid)

#x_grid, y_grid y z_grid son los valores interpolados


gridmin=np.min(z_grid)
gridmax=np.max(z_grid)


z_gridflat= z_grid.flatten()

# Combinar las coordenadas x_grid, y_grid y z_grid_interpolated en un vector
xyz_interpolated = np.column_stack((xiyi_mesh, z_gridflat))

# Crea el mapa de calor con interpolación Kriging Temperatura
plt.imshow(z_grid, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='GnBu')
plt.colorbar(label='Temperature, ºC')

  # Añade una barra de color para la escala de temperaturas
plt.scatter(x, y, facecolors='white', edgecolors='purple')  # Agrega los puntos de datos originales
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Kriging Heat map ')

# Muestra el mapa de calor
plt.savefig('Figure2Kriging.png', dpi=600)  #Cambie dpi a 600 Jun2024
plt.show()

# Crea el mapa de calor con interpolación Kriging
plt.imshow(ss, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='GnBu')
plt.colorbar(label='Semivarianza estimada')

  # Añade una barra de color para la escala de temperaturas
plt.scatter(x, y, facecolors='white', edgecolors='purple')  # Agrega los puntos de datos originales
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Heat map with Kriging interpolation')

# Muestra el mapa de calor
plt.savefig('Figura2KrigingVarianza.png', dpi=600)  #Cambie dpi a 600 Jun2024
plt.show()

# Commented out IPython magic to ensure Python compatibility.
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import f1_score
# %matplotlib inline
from sklearn.metrics import mean_squared_error, mean_absolute_error, mean_squared_log_error

# ESTE ES EL DE ERRORES PARA KRIGING EN TEMPERATURA
# Configura la validación cruzada (KFold)
#SUBgrupo train indices, train_indices

n_splits = 5  # Número de divisiones para la validación cruzada
kf = KFold(n_splits=n_splits, shuffle=True)

# Realiza la validación cruzada
for train_indices, test_indices in kf.split(coordinates):
    j= train_indices<1000
    k= test_indices<1000
    train_indices=train_indices[j]
    test_indices=test_indices[k]
    train_coords = coordinates[train_indices]

    test_coords = coordinates[test_indices]
#separa los coordinates en train group y test group

# Divide los datos de z_interpolated en conjuntos de entrenamiento y prueba
train_z = z_grid[train_indices, :]

#zgrid con train indices
trainz2= z_grid[train_indices,train_indices]

#zgrid con test indices
SUBtestz = z_grid[test_indices[:, np.newaxis], test_indices]


# Entrena el modelo Kriging con los datos de entrenamiento
#kriging_model = OrdinaryKriging(
 # train_coords[:, 0], train_coords[:, 1],train_z, variogram_model='linear'
 #   )
kriging_model = OrdinaryKriging(
  train_coords[:, 0], train_coords[:, 1],trainz2, variogram_model='linear'
    )


# Realiza la predicción en los puntos de prueba
predicted_z, _ = kriging_model.execute('grid', test_coords[:, 0], test_coords[:, 1])

# Evalúa la precisión del modelo (por ejemplo, usando el RMSE)
mse = mean_squared_error(SUBtestz, predicted_z, squared=False)
rmse = np.sqrt(mse)
mae = mean_absolute_error(SUBtestz, predicted_z)
print(f"Kriging Temperatura")
print(f"MSE for fold: {mse}")
print(f"RMSE for fold: {rmse}")
print(f"MAE for fold: {mae}")

#PRUEBAS PARa KRIGING: de Humedad no3 y potasio

# aqui hice lo de la concatenacion del grid. -funciona
# LUEGO: hacer que pueda restar z con zgrid para la posicion en x, y)



# Realiza la interpolación Kriging para humedad
OKw = OrdinaryKriging(x, y, w, variogram_model='linear', verbose=False)
w_grid, ss = OKw.execute('grid', x_grid, y_grid)
OKu = OrdinaryKriging(x, y, u, variogram_model='linear', verbose=False)
u_grid, ss = OKu.execute('grid', x_grid, y_grid)
OKv = OrdinaryKriging(x, y, v, variogram_model='linear', verbose=False)
v_grid, ss = OKv.execute('grid', x_grid, y_grid)

OKr = OrdinaryKriging(x, y, r, variogram_model='linear', verbose=False)
r_grid, ss = OKr.execute('grid', x_grid, y_grid)
OKs = OrdinaryKriging(x, y, s, variogram_model='linear', verbose=False)
s_grid, ss = OKs.execute('grid', x_grid, y_grid)

gridminw=np.min(w_grid)
gridmaxw=np.max(w_grid)
gridminr=np.min(r_grid)
gridmaxr=np.max(r_grid)
gridmins=np.min(s_grid)
gridmaxs=np.max(s_grid)

#x_grid, y_grid y z_grid son los valores interpolados

#Codigo usa mismos test y train de Temperatura
#separa los coordinates en train group y test group

# Divide los datos de z_interpolated en conjuntos de entrenamiento y prueba
train_w = w_grid[train_indices, :]
train_u = u_grid[train_indices, :]
train_v = v_grid[train_indices, :]
train_r= r_grid[train_indices, :]
train_s = s_grid[train_indices, :]
#zgrid con train indices
trainw2= w_grid[train_indices,train_indices]
trainu2= u_grid[train_indices,train_indices]
trainv2= v_grid[train_indices,train_indices]
trainr2= r_grid[train_indices,train_indices]
trains2= s_grid[train_indices,train_indices]

#zgrid con test indices
SUBtestw = w_grid[test_indices[:, np.newaxis], test_indices]
SUBtestu = u_grid[test_indices[:, np.newaxis], test_indices]
SUBtestv = v_grid[test_indices[:, np.newaxis], test_indices]
SUBtestr = r_grid[test_indices[:, np.newaxis], test_indices]
SUBtests = s_grid[test_indices[:, np.newaxis], test_indices]



# Entrena el modelo Kriging con los datos de entrenamiento
#kriging_model = OrdinaryKriging(
 # train_coords[:, 0], train_coords[:, 1],train_z, variogram_model='linear'
 #   )
kriging_modelw = OrdinaryKriging(
  train_coords[:, 0], train_coords[:, 1],trainw2, variogram_model='linear'
    )
kriging_modelu = OrdinaryKriging(
  train_coords[:, 0], train_coords[:, 1],trainu2, variogram_model='linear'
    )
kriging_modelv = OrdinaryKriging(
  train_coords[:, 0], train_coords[:, 1],trainv2, variogram_model='linear'
    )
kriging_modelr = OrdinaryKriging(
  train_coords[:, 0], train_coords[:, 1],trainr2, variogram_model='linear'
    )
kriging_models = OrdinaryKriging(
  train_coords[:, 0], train_coords[:, 1],trains2, variogram_model='linear'
    )


# Realiza la predicción en los puntos de prueba
predicted_w, _ = kriging_modelw.execute('grid', test_coords[:, 0], test_coords[:, 1])

predicted_u, _ = kriging_modelu.execute('grid', test_coords[:, 0], test_coords[:, 1])

predicted_v, _ = kriging_modelv.execute('grid', test_coords[:, 0], test_coords[:, 1])

predicted_r, _ = kriging_modelr.execute('grid', test_coords[:, 0], test_coords[:, 1])

predicted_s, _ = kriging_models.execute('grid', test_coords[:, 0], test_coords[:, 1])



# Evalúa la precisión del Humedad (por ejemplo, usando el RMSE)
mseWK = mean_squared_error(SUBtestw, predicted_w, squared=False)
rmseWK = np.sqrt(mseWK)
maeWK = mean_absolute_error(SUBtestw, predicted_w)
print(f"Error for Humedad-Kriging")
print(f"MSE for fold: {mseWK}")
print(f"RMSE for fold: {rmseWK}")
print(f"MAE for fold: {maeWK}")

# Evalúa la precisión del Nitrogeno (por ejemplo, usando el RMSE)
mseUK = mean_squared_error(SUBtestu, predicted_u, squared=False)
rmseUK = np.sqrt(mseUK)
maeUK = mean_absolute_error(SUBtestu, predicted_u)
print(f"Error for Nitrogeno mV-Kriging")
print(f"MSE for fold: {mseUK}")
print(f"RMSE for fold: {rmseUK}")
print(f"MAE for fold: {maeUK}")

# Evalúa la precisión del Potasio (por ejemplo, usando el RMSE)
msevK = mean_squared_error(SUBtestv, predicted_v, squared=False)
rmsevK = np.sqrt(msevK)
maevK = mean_absolute_error(SUBtestv, predicted_v)
print(f"Error for Potasio mV-Kriging")
print(f"MSE for fold: {msevK}")
print(f"RMSE for fold: {rmsevK}")
print(f"MAE for fold: {maevK}")

# Evalúa la precisión del Nitrogeno en mM(por ejemplo, usando el RMSE)
mserK = mean_squared_error(SUBtestr, predicted_r, squared=False)
rmserK = np.sqrt(mserK)
maerK = mean_absolute_error(SUBtestr, predicted_r)
print(f"Error for Nitrogeno mMcal-Kriging LogM")
print(f"MSE for fold: {mserK}")
print(f"RMSE for fold: {rmserK}")
print(f"MAE for fold: {maerK}")

# Evalúa la precisión del Potasio (por ejemplo, usando el RMSE)
msesK = mean_squared_error(SUBtests, predicted_s, squared=False)
rmsesK = np.sqrt(msesK)
maesK = mean_absolute_error(SUBtests, predicted_s)
print(f"Error for Potasio mMcal-Kriging LogM")
print(f"MSE for fold: {msesK}")
print(f"RMSE for fold: {rmsesK}")
print(f"MAE for fold: {maesK}")



# Crea el mapa de calor con interpolación Kriging Humedad
plt.imshow(w_grid, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='GnBu', vmin=gridminw, vmax=gridmaxw)
plt.colorbar(label='Soil moisture %')

  # Añade una barra de color para la escala de temperaturas
plt.scatter(x, y, facecolors='white', edgecolors='purple')  # Agrega los puntos de datos originales
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Soil Moisture Kriging heatmap')

# Muestra el mapa de calor
plt.savefig('Figure2KrigingMoisturelog.png', dpi=600)  #Cambie dpi a 600 Jun2024
plt.show()



# Crea el mapa de calor con interpolación Kriging Nitrogeno
plt.imshow(r_grid, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='GnBu', vmin=gridminr, vmax=gridmaxr)
plt.colorbar(label='NiO3- (log(M))')

  # Añade una barra de color para la escala de temperaturas
plt.scatter(x, y, facecolors='white', edgecolors='purple')  # Agrega los puntos de datos originales
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('NiO3 Kriging heatmap')

# Muestra el mapa de calor
plt.savefig('Figure2KrigingNO3logM.png', dpi=600)  #Cambie dpi a 600 Jun2024
plt.show()



# Crea el mapa de calor con interpolación Kriging Potasio
plt.imshow(s_grid, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='GnBu', vmin=gridmins, vmax=gridmaxs)
plt.colorbar(label='K+ (log(M))')

  # Añade una barra de color para la escala de temperaturas
plt.scatter(x, y, facecolors='white', edgecolors='purple')  # Agrega los puntos de datos originales
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('K+ Kriging Heatmap')

# Muestra el mapa de calor
plt.savefig('Figure2KrigingKlogM.png', dpi=600)  #Cambie dpi a 600 Jun2024
plt.show()

#esta va. ESTA ES LA FIGURA para IDW

from scipy.interpolate import Rbf

# Definir la función de interpolación
rbfz = Rbf(x, y, z, function='linear')
rbfw = Rbf(x, y, w, function='linear')
rbfu = Rbf(x, y, u, function='linear')
rbfv = Rbf(x, y, v, function='linear')
rbfr = Rbf(x, y, r, function='linear')
rbfs = Rbf(x, y, s, function='linear')

# Generar una cuadrícula para la interpolación
#xi2, yi2 = np.meshgrid(np.linspace(0, 2, 100), np.linspace(0, 1, 100))
xi2 = np.linspace(x_min, x_max, grid_resolution)
yi2 = np.linspace(y_min, y_max, grid_resolution)

cbar_min=gridmin
cbar_max=gridmax

xi3, yi3 = np.meshgrid(xi2, yi2)

#make flat, put into 1 vector coordinates de 1000000x2
# Combinación de las coordenadas x_grid e y_grid en una malla 2D
xiyi_meshIDW = np.column_stack((xi3.ravel(), yi3.ravel()))
coordinatesIDW=xiyi_meshIDW

# Interpolar los valores en la cuadrícula
zi3 = rbfz(xi3, yi3)
wi3 = rbfw(xi3, yi3)
ui3 = rbfu(xi3, yi3)
vi3 = rbfv(xi3, yi3)
ri3 = rbfr(xi3, yi3)
si3 = rbfs(xi3, yi3)

cbar_min=gridmin
cbar_max=gridmax

# Graficar los datos originales y la interpolación
plt.scatter(x, y, facecolors='white', edgecolors='purple')
plt.imshow(zi3, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='GnBu', clim=(cbar_min, cbar_max))
plt.colorbar(label='Temperature, ºC')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Heat map with IDW interpolation')
plt.savefig('Figure 3 IDW.png', dpi=600)   #Cambie dpi a 600 Jun2024
plt.show()

cbar_minr=gridminr
cbar_maxr=gridmaxr
# Graficar los datos originales y la interpolación
plt.scatter(x, y, facecolors='white', edgecolors='purple')
plt.imshow(ri3, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='GnBu', clim=(cbar_minr, cbar_maxr))
plt.colorbar(label='NiO3- (log (M))')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('NiO3- IDW heatmap')
plt.savefig('Figure 3 IDWNio3logM.png', dpi=600)   #Cambie dpi a 600 Jun2024
plt.show()
cbar_mins=gridmins
cbar_maxs=gridmaxs
# Graficar los datos originales y la interpolación
plt.scatter(x, y, facecolors='white', edgecolors='purple')
plt.imshow(si3, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='GnBu', clim=(cbar_mins, cbar_maxs))
plt.colorbar(label='K+ (log (M))')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('K+ IDW heatmap')
plt.savefig('Figure 3 IDWKlogM.png', dpi=600)   #Cambie dpi a 600 Jun2024
plt.show()

# aqui se hace el error de IDW TEMPERATURA
# Configura la validación cruzada (KFold)
#SUBgrupo train indices, train_indices

n_splits = 5  # Número de divisiones para la validación cruzada
kf = KFold(n_splits=n_splits, shuffle=True)

# Realiza la validación cruzada
for train_indices, test_indices in kf.split(coordinatesIDW):
    j= train_indices<1000
    k= test_indices<1000
    train_indices=train_indices[j]
    test_indices=test_indices[k]
    train_coords = coordinatesIDW[train_indices]
    test_coords = coordinatesIDW[test_indices]
#separa los coordinates en train group y test group

# Divide los datos de z_interpolated en conjuntos de entrenamiento y prueba
train_z = zi3[train_indices, :]
train_w = wi3[train_indices, :]
train_u = ui3[train_indices, :]
train_v = vi3[train_indices, :]
train_r = ri3[train_indices, :]
train_s = si3[train_indices, :]

#zgrid con train indices
trainz2= zi3[train_indices,train_indices]
trainw2= wi3[train_indices,train_indices]
trainu2= ui3[train_indices,train_indices]
trainv2= vi3[train_indices,train_indices]
trainr2= ri3[train_indices,train_indices]
trains2= si3[train_indices,train_indices]



#zgrid con test indices
SUBtestz = zi3[test_indices[:, np.newaxis], test_indices]
SUBtestw = wi3[test_indices[:, np.newaxis], test_indices]
SUBtestu = ui3[test_indices[:, np.newaxis], test_indices]
SUBtestv = vi3[test_indices[:, np.newaxis], test_indices]
SUBtestr = ri3[test_indices[:, np.newaxis], test_indices]
SUBtests = si3[test_indices[:, np.newaxis], test_indices]

# Entrena el modelo IDW con los datos de entrenamiento
# Entrena la función de interpolación

rbfztrain = Rbf(train_coords[:, 0], train_coords[:, 1], trainz2, function='linear')
rbfwtrain = Rbf(train_coords[:, 0], train_coords[:, 1], trainw2, function='linear')
rbfutrain = Rbf(train_coords[:, 0], train_coords[:, 1], trainu2, function='linear')
rbfvtrain = Rbf(train_coords[:, 0], train_coords[:, 1], trainv2, function='linear')
rbfrtrain = Rbf(train_coords[:, 0], train_coords[:, 1], trainr2, function='linear')
rbfstrain = Rbf(train_coords[:, 0], train_coords[:, 1], trains2, function='linear')

newx=test_coords[:, 0]
newy=test_coords[:, 1]
xi4, yi4 = np.meshgrid(newx, newy)
# Interpolar los valores en la cuadrícula
zinterpolada = rbfztrain(xi4, yi4)
winterpolada = rbfwtrain(xi4, yi4)
uinterpolada = rbfutrain(xi4, yi4)
vinterpolada = rbfvtrain(xi4, yi4)
rinterpolada = rbfrtrain(xi4, yi4)
sinterpolada = rbfstrain(xi4, yi4)


# Evalúa la precisión del modelo (por ejemplo, usando el RMSE)
mseIDWz = mean_squared_error(SUBtestz, zinterpolada, squared=False)
rmseIDWz = np.sqrt(mseIDWz)
maeIDWz = mean_absolute_error(SUBtestz, zinterpolada)
print(f"Error for Temperatura-IDW")
print(f"MSE for fold: {mseIDWz}")
print(f"RMSE for fold: {rmseIDWz}")
print(f"MAE for fold: {maeIDWz}")

mseIDWw = mean_squared_error(SUBtestw, winterpolada, squared=False)
rmseIDWw = np.sqrt(mseIDWw)
maeIDWw = mean_absolute_error(SUBtestw, winterpolada)
print(f"Error for Humedad-IDW")
print(f"MSE for fold: {mseIDWw}")
print(f"RMSE for fold: {rmseIDWw}")
print(f"MAE for fold: {maeIDWw}")

mseIDWu = mean_squared_error(SUBtestu, uinterpolada, squared=False)
rmseIDWu = np.sqrt(mseIDWu)
maeIDWu = mean_absolute_error(SUBtestu, uinterpolada)
print(f"Error for Nitrogeno-IDW")
print(f"MSE for fold: {mseIDWu}")
print(f"RMSE for fold: {rmseIDWu}")
print(f"MAE for fold: {maeIDWu}")

mseIDWv = mean_squared_error(SUBtestv, vinterpolada, squared=False)
rmseIDWv = np.sqrt(mseIDWv)
maeIDWv = mean_absolute_error(SUBtestv, vinterpolada)
print(f"Error for Potasio-IDW")
print(f"MSE for fold: {mseIDWv}")
print(f"RMSE for fold: {rmseIDWv}")
print(f"MAE for fold: {maeIDWv}")

mseIDWr = mean_squared_error(SUBtestr, rinterpolada, squared=False)
rmseIDWr = np.sqrt(mseIDWr)
maeIDWr = mean_absolute_error(SUBtestr, rinterpolada)
print(f"Error for Nitrogeno mMcal-IDW")
print(f"MSE for fold: {mseIDWr}")
print(f"RMSE for fold: {rmseIDWr}")
print(f"MAE for fold: {maeIDWr}")

mseIDWs = mean_squared_error(SUBtests, sinterpolada, squared=False)
rmseIDWs = np.sqrt(mseIDWs)
maeIDWs = mean_absolute_error(SUBtests, sinterpolada)
print(f"Error for Potasio mMcal-IDW")
print(f"MSE for fold: {mseIDWs}")
print(f"RMSE for fold: {rmseIDWs}")
print(f"MAE for fold: {maeIDWs}")



# CALCULO DE DISTANCIA ENTRE DOS PUNTOS CARDINALES, decimal
import math
# hecho para puntos: 0, 6, 60, 64
def haversine(lat1, lon1, lat2, lon2):
    # Convertir grados a radianes
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Diferencias de latitud y longitud
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Aplicar la fórmula de Haversine
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))

    # Radio de la Tierra en kilómetros (puede variar ligeramente según la fuente)
    r = 6371

    # Calcular la distancia
    distance = c * r

    return distance

# Ejemplo de uso
lat1 = 3.9153567# Latitud de Nueva York
lon1 = -74.9841224 # Longitud de Nueva York
lat2 =3.91491114 # Latitud de Los Ángeles
lon2 =-74.9842075# Longitud de Los Ángeles

distancia = haversine(lat1, lon1, lat2, lon2)*1000 #en metros
print(f"La distancia entre Lat1 y lat2 es de {distancia:.2f} m")

#Codigo para calcular area dados 4 puntos cardinales.
import numpy as np
from geopy.distance import distance


def calculate_area(latitudes, longitudes):
    """
    Calculate the area of a polygon given its vertices defined by latitudes and longitudes.
    The coordinates should be given in decimal degrees.

    Args:
    latitudes (list): List of latitudes in decimal degrees.
    longitudes (list): List of longitudes in decimal degrees.

    Returns:
    float: Area of the polygon in square meters.
    """

    # Ensure the polygon is closed by adding the first point at the end
    if latitudes[0] != latitudes[-1] or longitudes[0] != longitudes[-1]:
        latitudes.append(latitudes[0])
        longitudes.append(longitudes[0])

    # Convert lat/lon to radians
    latitudes = np.radians(latitudes)
    longitudes = np.radians(longitudes)

    # Radius of the Earth in meters
    R = 6378137

    # Calculate the area using the spherical excess formula
    area = 0.0
    for i in range(len(latitudes) - 1):
        lat1 = latitudes[i]
        lon1 = longitudes[i]
        lat2 = latitudes[i + 1]
        lon2 = longitudes[i + 1]

        # Calculate the spherical excess
        area += (lon2 - lon1) * (2 + np.sin(lat1) + np.sin(lat2))

    # Absolute value of the result divided by 2
    area = np.abs(area) * (R**2) / 2.0

    return area

# Ejemplo de uso
latitudes = [3.9151008, 3.91491114, 3.9152408, 3.9154275]
longitudes = [-74.9844771, -74.9842075, -74.9839388, -74.9842298]

area = calculate_area(latitudes, longitudes)
print(f"Área del polígono: {area:.2f} metros cuadrados")



#Grafico de x vsy con error.
#Solo plot NO3-
import matplotlib.pyplot as plt
import numpy as np

# Datos de TABLa 1, no3
x = np.array([4, 7, 10])  #concentracion
y1 = np.array([3.49, 9.73, 46.24]) #isfet
y2 = np.array([3.87, 8.38, 48.38]) #horiba
# Errores
y1_error = np.array([0.0981, 0.1611, 0.0442])
# Crear el gráfico
plt.figure(figsize=(10, 6))
#plt.plot(y2, y1, 'o-', label='Serie 1 (Segunda columna)')
#plt.plot(y2, y2, 's-', label='Serie 2 (Tercera columna)')

plt.errorbar(y2, y1, yerr=y1_error, fmt='o-', label='Serie 1 (Segunda columna)')
#plt.errorbar(y, y2, yerr=y2_error, fmt='s-', label='Serie 2 (Tercera columna)')

# Etiquetas y título
plt.xlabel('X (Primera columna)')
plt.ylabel('Y (Segunda y Tercera columnas)')
plt.title('Gráfico de Datos Proporcionados NO3-')
plt.legend()
plt.grid(True)

# Mostrar el gráfico
plt.show()

#Grafico de x vsy con error.
#Solo plot k+
import matplotlib.pyplot as plt
import numpy as np

# Datos de TABLa 1, k+
x = np.array([4, 7, 10])  #concentracion
y1 = np.array([4.47, 13.58, 47.35]) #isfet
y2 = np.array([5.37, 10.48, 51.15]) #horiba
# Errores
y1_error = np.array([0.0981, 0.1611, 0.0442])
# Crear el gráfico
plt.figure(figsize=(10, 6))
#plt.plot(y2, y1, 'o-', label='Serie 1 (Segunda columna)')
#plt.plot(y2, y2, 's-', label='Serie 2 (Tercera columna)')

plt.errorbar(y2, y1, yerr=y1_error, fmt='o-', label='Serie 1 (Segunda columna)')
#plt.errorbar(y, y2, yerr=y2_error, fmt='s-', label='Serie 2 (Tercera columna)')

# Etiquetas y título
plt.xlabel('X (Primera columna)')
plt.ylabel('Y (Segunda y Tercera columnas)')
plt.title('Gráfico de Datos Proporcionados')
plt.legend()
plt.grid(True)

# Mostrar el gráfico
plt.show()

#CAALIBRAR DATOS CON HORIBA
#En paper
#K+
#todas las veces sale igual.
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

# Datos de ejemplo de los dos sensores
horiba = np.array([5.37, 10.48, 51.15])# Sensor calibrado
isfet = np.array([4.47, 13.58, 47.35]) # Sensor sin calibrar


# Datos de TABLa 1, no3
x1 = np.array([4, 7, 10]) #concentracion
# Errores
isfet_error = np.array([-0.0981, 0.1611, -0.0442])

# Realizar la regresión lineal para ajustar isfet a horiba
slope, intercept, r_value, p_value, std_err = stats.linregress(isfet, horiba)

# Calcular los valores recalibrados del sensor sin calibrar
isfet_calibrated = slope * isfet + intercept

# Graficar los datos originales y los recalibrados
plt.figure(figsize=(10, 6))
plt.scatter(isfet, horiba, label='Datos Originales K+', color='blue')
plt.plot(isfet, isfet_calibrated, 'r-', label='Ajuste Lineal')
plt.xlabel('Sensor sin Calibrar (isfet)')
plt.ylabel('Sensor Calibrado (horiba)')
plt.title('Ajuste Lineal para Recalibrar Sensor K+')
plt.legend()
plt.grid(True)
plt.show()

print(f"Slope: {slope}, Intercept: {intercept}, R-squared: {r_value**2}")

# Mostrar los valores recalibrados
print("Valores del sensor sin calibrar (isfet):", isfet)
print("Valores recalibrados del sensor sin calibrar:", isfet_calibrated)

#CAALIBRAR DATOS CON HORIBA No3-
#En paper
#todas las veces sale igual.
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

# Datos de ejemplo de los dos sensores
horiba = np.array([3.87, 8.38, 48.38]) # Sensor calibrado
isfet = np.array([3.49, 9.73, 46.24])   # Sensor sin calibrar

# Datos de TABLa 1, no3
x1 = np.array([4, 7, 10]) #concentracion
# Errores
isfet_error = np.array([-0.0981, 0.1611, -0.0442])

# Realizar la regresión lineal para ajustar isfet a horiba
slope, intercept, r_value, p_value, std_err = stats.linregress(isfet, horiba)

# Calcular los valores recalibrados del sensor sin calibrar
isfet_calibrated = slope * isfet + intercept

# Graficar los datos originales y los recalibrados
plt.figure(figsize=(10, 6))
plt.scatter(isfet, horiba, label='Datos Originales', color='blue')
plt.plot(isfet, isfet_calibrated, 'r-', label='Ajuste Lineal')
plt.xlabel('Sensor sin Calibrar (isfet)')
plt.ylabel('Sensor Calibrado (horiba)')
plt.title('Ajuste Lineal para Recalibrar Sensor')
plt.legend()
plt.grid(True)
plt.show()

print(f"Slope: {slope}, Intercept: {intercept}, R-squared: {r_value**2}")

# Mostrar los valores recalibrados
print("Valores del sensor sin calibrar (isfet):", isfet)
print("Valores recalibrados del sensor sin calibrar:", isfet_calibrated)

#CAALIBRAR DATOS CON HORIBA phSensor
#En paper
#todas las veces sale igual.
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

# Datos de ejemplo de los dos sensores
horiba = np.array([4.1, 7.1, 9.3]) # Sensor calibrado
isfet = np.array([3.98, 7.12, 9.38])   # Sensor sin calibrar

# Datos de TABLa 1, no3
x1 = np.array([4, 7, 10]) #concentracion
# Errores
isfet_error = np.array([-0.0292, 0.0028, -0.0086])

# Realizar la regresión lineal para ajustar isfet a horiba
slope, intercept, r_value, p_value, std_err = stats.linregress(isfet, horiba)

# Calcular los valores recalibrados del sensor sin calibrar
isfet_calibrated = slope * isfet + intercept

# Graficar los datos originales y los recalibrados
plt.figure(figsize=(10, 6))
plt.scatter(isfet, horiba, label='Datos Originales', color='blue')
plt.plot(isfet, isfet_calibrated, 'r-', label='Ajuste Lineal')
plt.xlabel('Sensor sin Calibrar (isfet)')
plt.ylabel('Sensor Calibrado (horiba)')
plt.title('Ajuste Lineal para Recalibrar Sensor pH ')
plt.legend()
plt.grid(True)
plt.show()

print(f"Slope: {slope}, Intercept: {intercept}, R-squared: {r_value**2}")

# Mostrar los valores recalibrados
print("Valores del sensor sin calibrar (isfet):", isfet)
print("Valores recalibrados del sensor sin calibrar:", isfet_calibrated)

