from settings_thermals import *


list_of_totalenergy = []
list_of_derive_totalenergy = []
abscisse, abscisse2, abscisse3 = [], [], []
if __name__ == '__main__':
    
    #Faire une boucle while
    
    ivy = IvyMessagesInterface("DemoSettings")
    

    setting_manager = PprzSettingsManager(sys.argv[1], sys.argv[2], ivy)
    speed = setting_manager["airspeed"]
    altitude = setting_manager["alt"]
    

    value = 0.5 * (MASSE) * speed * speed + (MASSE) * 9.81 * altitude
    list_of_totalenergy.append(value)
    taille = len(list_of_totalenergy)
    if taille >= 2:
        derive = (list_of_totalenergy[taille - 1] - list_of_totalenergy[taille - 2]) / (abscisse[taille - 1] - abscisse[taille - 2])
        list_of_totalenergy.append(derive)
        abscisse2.append((abscisse[taille - 1] + abscisse[taille - 2]) / 2)
        taille2 = len(list_of_totalenergy)
        
        if taille2 >= 2:
            derive2 = (list_of_totalenergy[taille2 - 1] - list_of_totalenergy[taille2 - 2]) / (abscisse2[taille2 - 1] - abscisse2[taille2 - 2])
            P.append(derive2)
            abscisse3.append((abscisse[taille - 1] + abscisse[taille - 2]) / 2)
            
            #Cas rotation horaire
            if derive2 <= 0:
                bankangle = 50
            else:
                bankangle = 17.5
                
            setting_manager["roll"] = bankangle
        
    
    
