# Methode


Nous décomposerons l'étude du bras final en plusieurs sous-parties, en partant d'un système comprenant 2 degrés de libertés et en en ajoutant un par un jusqu'à atteindre le nombre de degrés de libertés du bras réel. 
Nous espérons ainsi pouvoir identifier à chaque étape les difficultés additionnels liés à chaque ajout. 

Pour chaque modèle, nous utiliserons la convention de [Denavit-Hartenberg](https://fr.wikipedia.org/wiki/Denavit-Hartenberg) pour visualiser de façon schématique le système. 
Ensuite, nous définirons les matrices de transition pour chaque liaison, même si dans les premiers cas d'étude une approche graphique pourrait amener à une solution plus rapide. 

Enfin, nous préciserons pour chaque système une solution, relié à un script python puis une implémentation en Rust sur un système réel.