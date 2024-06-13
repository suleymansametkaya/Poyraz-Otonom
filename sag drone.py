# Description: IHA'nin belirtilen irtifaya yukselmesi ve belirtilen alanin taranmasi

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
from pymavlink import mavutil

# Drone Baglantisi icin ip ve port secimi
dronebaglanti = "127.00.1:14550"

# Drone baglantisi yapiliyor
sagdrone = connect(f'{dronebaglanti}', wait_ready=True)

# IHA'nin arm edilmesi ve belirtilen irtifaya yukselmesi
def takeoff(irtifa):

    # IHA'nin arm edilebilir duruma gelesiye kadar bekleniyor 
    while sagdrone.is_armable is not True:
        print("İHA arm edilebilir durumda değil.")
        time.sleep(1)


    print("İHA arm edilebilir.")

    # IHA GUIDED moduna aliniyor
    sagdrone.mode = VehicleMode("GUIDED")

    # IHA arm ediliyor
    sagdrone.armed = True

    # IHA arm edilene kadar bekleniyor
    while sagdrone.armed is not True:
        print("İHA arm ediliyor...")
        time.sleep(0.5)

    print("İHA arm edildi.")

    # IHA belirtilen irtifaya yukseltiliyor
    sagdrone.simple_takeoff(irtifa)
    
    # IHA belirtilen irtifaya yukselene kadar bekleniyor
    while sagdrone.location.global_relative_frame.alt < irtifa * 0.9:
        print("İha hedefe yükseliyor.")
        time.sleep(1)


# takeoff(7)

# IHA'nin hiz ve konum bilgileri
sagdrone.airspeed = 2
sagdrone.groundspeed = 2
sagdrone.velocity[0] =2
sagdrone._yawspeed=2
sagdrone._pitchspeed=2


# -35.3632019, 149.1649845, -35.3631713, 149.1659769, -35.3621192, 149.1658482, -35.3621257, 149.1648799
#       X1,         Y1,          X2,         Y2,             X3,         Y3,            X4,      Y4


# Alanin taranmasi icin belirlenen noktalarin koordinatlarinin alinmasi ve IHA'nin bu noktalari tarayacak sekilde hareket etmesi
def tara(x1,y1,x2,y2,x3,y3,x4,y4):
    x_orta1 = (x1+x4)/2
    x_orta2 = (x1+x_orta1)/2
    x_orta3 = (x4+x_orta1)/2
    y_orta1 = (y1+y2)/2
    
    global komut
    komut = sagdrone.commands
    komut.clear()
    time.sleep(1)
    
    
    #takeoff
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 7))#cmd0
    
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0,0, x2, y_orta1, 7))#cmd1
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))#cmd2
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0,0,x2, y2, 7))#cmd3

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0,0,0, x_orta2, y2, 7))#cmd4
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))#cmd5
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0,0,0, x_orta2, y_orta1, 7))#cmd6

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0,0,0, x_orta1, y_orta1, 7))#cmd7
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))#cmd8
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0,0,0, x_orta1, y2, 7))#cmd9

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0,0,0, x_orta3, y2, 7))#cmd10
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))#cmd11
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0,0,0, x_orta3, y_orta1, 7))#cmd12

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0,0,0, x4, y_orta1, 7))#cmd13
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 0, 1, 0, 0, 0, 0, 0, 0))#cmd14
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0,0,0, x3, y3, 7))#cmd15

    #rtl ---> iha rtl yaptığında irtifa 15 metre ye çıkartıyo 
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))#cmd16
    #gorev bitiş doğrulama
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))#cmd17
    
    komut.upload()
    print("Komutlar yukleniyor.\n")
    
    
takeoff(10)    
# -35,3631429 ,149,1645849, -35,3631429 ,149,1656604, -35,3621104, 149,1656604,  -35,3621104, 149,1645849
# x1, x4,

# -35.3632019, 149.1649845, -35.3631713, 149.1659769, -35.3621192, 149.1658482, -35.3621257, 149.1648799
#       X1,         Y1,          X2,         Y2,             X3,         Y3,            X4,      Y4


tara(-35.3632019, 149.1649845, -35.3631713, 149.1659769, -35.3621192, 149.1658482, -35.3621257, 149.1648799)
    
    
komut.next = 0
# IHA AUTO moduna aliniyor
sagdrone.mode = VehicleMode("AUTO")
time.sleep(1)
print(f"İha : " + str(sagdrone.mode) + " moduna alindi.\n")

# IHA'nin belirtilen komutlari gerceklestirmesi
while True:
    
    sagdrone.airspeed = 2
    sagdrone.groundspeed = 2
    sagdrone.velocity[0] =2

    next_komut = komut.next
    print(f"Siradaki komut : {next_komut}\n")
    time.sleep(2.5)
    print(f"Konum X : " + str(sagdrone.location.global_relative_frame.lat))
    print(f"Konum Y : " + str(sagdrone.location.global_relative_frame.lon))
    print(f"Konum Z : " + str(sagdrone.location.global_relative_frame.alt))
    print("air speed : " + str(sagdrone.airspeed))
    print("ground speed : " + str(sagdrone.groundspeed))
    print("----------------------------")
   
    if next_komut is 10:
        print("Alanin yarisi tarandi\n")
        print("Alanin yarisi tarandi\n")


    time.sleep(1)
    if next_komut is 13:
        print(" Alan Tarama Gorevi Bitti\n")
        print(" Alan Tarama Gorevi Bitti\n")
        print(" Alan Tarama Gorevi Bitti\n")
        

    if next_komut is 16:

        print("RTL yapiliyor...\n")
        time.sleep(2.5)

    if next_komut is 17:

        print("RTL Bitti\n")
        time.sleep(1)
        break