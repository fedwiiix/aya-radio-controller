/*
if(led!=0){     // led de notification

     if(j>1020)
        j=0;
      // variation de rouge
      if (j<=255) { // défile valeur 0 à 255
        ledRVBpwm(j,0,0); // génère impulsion largeur voulue pour la couleur
      }
      // variation de bleu - rouge dégressif
      if ( j>255 && j<=510 ) { // défile valeur 0 à 255
        ledRVBpwm(510-j,0,j-255); // génère impulsion largeur voulue pour la couleur
      }
      // variation de vert - bleu dégressif
      if (j>510 && j<=765) { // défile valeur 0 à 255
        ledRVBpwm(0,j-510,510-j); // génère impulsion largeur voulue pour la couleur
      }
      // variation de jaune 
      if (j>765 && j<=1020) { // défile valeur 0 à 255
        ledRVBpwm(j-765,255,0); // génère impulsion largeur voulue pour la couleur
      }
      
      delay(10); //pause
      j++;
      
}
*/
