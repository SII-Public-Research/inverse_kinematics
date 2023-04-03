# Introduction

Ce livre a pour but de présenter la démarche expérimentale que nous avons mise en oeuvre afin de maîtriser, ou du moins mieux apréhender les techniques de cinématique inverse utilisées en robotique. 

Le but final étant de pouvoir contrôler de manière précise un bras robotique comprenant 6 degrées de libertés. 


# Outils

Au sein de notre équipe, nous avons un petit faible pour le language RUST et son implémentation dans des cibles embarqués. 
Nous avons donc souhaité utilisé ce language pour la partie embarquée.
Nous utiliserons également le Python pour tester les approches mathématiques mises en avant afin de rapidement pouvoir modéliser un système. 


# Matériel 

Notre étude se voulant pratique, nous sommes parti sur un bras robotique disponible dans le commerce.
C'est le 
Les différents liens utiles étant :
* [Achat](https://www.gotronic.fr/art-bras-robotique-joy-it-robot02-26637.html)
* [Montage](https://www.gotronic.fr/pj2-robot02-guide-de-montage-1556.pdf)


Afin de contrôler le bras, nous avons détéré un vieux shield moteur que nous avions déjà utilisé auparavent, en ajoutant des fonctionnalités 