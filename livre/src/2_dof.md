# 2 dof robotic arm

Commençons par monter uniquement deux servomoteurs sur notre bras. 
En respectant les conventions de **Denavit-Hartenberg**, nous pouvons construire le schéma représentatif :
![schema_2dof](images/2dof_luna_arm.drawio.png)

Une fois la position initiale du robot choisie, ici un angle de 90° entre le segment **OA** et **AM**, nous pouvons écrire les matrices de transitions correspondantes :

$$
T01 = 
\left(\begin{array}{cc}
C1 & 0 & S1 & 0\\\\
S1 & 0 & -C1 & 0\\\\
0 & 1 & 0 & a_1\\\\
0 & 0 & 0 & 1
\end{array}\right)
$$

$$
T12 = 
\left(\begin{array}{cc}
C2 & -S2 & 0 & a_2C2\\\\
S2 & C2 & 0 & a_2S2\\\\
0 & 0 & 1 & 0\\\\
0 & 0 & 0 & 1
\end{array}\right)
$$


Avec $$ S1 = sin(θ_1), S2 = sin(θ_2), C1 = cos(θ_1), C2 = cos(θ_2) $$


Une fois les matrices de transitions pour chaque liaisons réalisés, nous pouvons faire leur produit matriciel afin d'obtenir la matrice caractéristique du système étudié. 

$$
T02 = 
\left(\begin{array}{cc}
C1C2 & -C1S2 & S1 & a_2C1C2\\\\
S1C2 & -S1S2 & -C1 & a_2S1C2\\\\
S2 & C2 & 0 & a_1 + a_2S2\\\\
0 & 0 & 0 & 1
\end{array}\right)
$$

Dans ce cas précis, avec uniquement deux liaisons, le système est relativement simple et nous pouvons nous contenter d'utiliser la dernière colonne de la matrice qui représente les équations de la position finale du bras en fonction des angles des moteurs.

$$ 
\begin{cases}
    x = a_2 * cos(θ_1) * cos(θ_2) \\\\
    y = a_2 * sin(θ_1) * cos(θ_2) \\\\
    z = a_1 + a_2 * sin(θ_2)
\end{cases}
$$

Pour **θ<sub>1</sub>**, nous pouvons calculer son sinus et son cosinus à partir des équations de Y et X :
$$
\begin{cases}
    sin(θ_1)  = \cfrac{y}{a_2 * cos(θ_2)}\\\\
    cos(θ_1) = \cfrac{x}{a_2 * cos(θ_2)}
\end{cases}
$$

on obtient ainsi **θ<sub>1</sub>** en faisant le calcul eq(y) / eq(x) :

$$
θ_1 = atan2(\cfrac{y}{x})
$$

Pour **θ<sub>2</sub>**, nous pouvons calculer son sinus et son cosinus à partir des équations de Z et X (ou Y) :
$$
\begin{cases}
sin(θ_2) = \cfrac{z - a_1}{a_2}\\\\
cos(θ_2) = \cfrac{x}{a_2 * cos(θ_1)}
\end{cases}
$$


Enfin, nous pouvons obtenir l'angle **θ<sub>2</sub>** de la même manière que **θ<sub>1</sub>** :

$$
θ_2 = atan2(\cfrac{(z - a_1) * cos(θ_1)}{x})
$$

Nous pouvons observer qu'il est relativement facile dans le cas de deux liaisons d'obtenir une cinématique inverse du système. 

En revanche, le système est limité et ne peut atteindre qu'un champ réduit de points, correspondant à une sphère de rayon a2. 
Nous notons également que dans notre cas d'étude, les servomoteurs ont une portée comprise entre 0 et 180°.

Afin de rendre le programme utilisable facilement, nous avons donc prévu une correction de la commande en Z, qui sera calculé à partir des paramètres d'entrés (X, Y). Cela évite de devoir calculer précisément une position du bras pour obtenir un déplacement réalisable. 

$$
z = \sqrt{a_2^2 - x^2 - y^2} + a_1
$$



