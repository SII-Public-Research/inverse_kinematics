# Introduction

Ce livre a pour but de présenter la démarche expérimentale que nous avons mise en œuvre afin de maîtriser, ou du moins mieux appréhender, les techniques de cinématiques inverses utilisées en robotique. 

Le but final étant de pouvoir contrôler de manière précise un bras robotique comprenant 5 degrés de libertés. 


# Outils

Au sein de notre équipe, nous avons un petit faible pour le langage RUST et son implémentation dans des cibles embarqués. 
Nous avons donc souhaité utiliser ce langage pour la partie embarquée.

Du Python sera utilisé pour tester les approches mathématiques mises en avant afin de rapidement modéliser un système. Les principales librairies utilisées étant **numpy** et **sympy**

Enfin, nous créerons un lien entre la commande moteur et les algorithmes en utilisant l’environnement ROS (robot operating system). 

# Matériel

Notre étude se voulant pratique, nous sommes parti sur un bras robotique disponible dans le commerce.

Les différents liens utiles étant :
* [Achat](https://www.gotronic.fr/art-bras-robotique-joy-it-robot02-26637.html)
* [Montage](https://www.gotronic.fr/pj2-robot02-guide-de-montage-1556.pdf)


Afin de contrôler le bras, nous avons déterré un vieux shield moteur que nous avions déjà utilisé auparavant, en ajoutant des fonctionnalités de contrôle de servomoteurs.
Les différents liens utiles étant :
* [Présentation](https://www.makerbuying.com/docs/new-sna41)
* [crate RUST](https://github.com/SII-Public-Research/sna41-motorshield)

