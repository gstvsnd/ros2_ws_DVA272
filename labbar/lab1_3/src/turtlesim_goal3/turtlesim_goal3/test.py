
waypoints = []
temp_string = ""
waypoints_string = "1.0, 22.0; 4.2, 5.4;"

for i in range(len(waypoints_string)):
    if waypoints_string[i] == ',' or waypoints_string[i] == ';':
        if waypoints_string[i-4] == ' ':
            temp_stringlist = [waypoints_string[i-3], waypoints_string[i-2], waypoints_string[i-1]]
        else:    
            temp_stringlist = [waypoints_string[i-4], waypoints_string[i-3], waypoints_string[i-2], waypoints_string[i-1]]
        temp_string = ''.join(temp_stringlist)#Här gör vi en lista med trängar till en sträng
        waypoints.append(float(temp_string))
print(waypoints)