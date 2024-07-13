# Object detection

This document is dedicated to the Yolo Object Detection system.

graph TD;
    A[Base de données d'objets connus (x,y,z,h,w,p)] -->|or| D[Algorithme]
    B[Base de données d'objets de grande taille estimée (h,w,p)] --> D
    C[YOLO reconnaissance d'objets (x,y,h,w)] --> D
    D --> E[Repositionnement du drone]
    D --> F[Carte sémantique]
    G[Paramètre intrinsèque Caméra] --> D
    end