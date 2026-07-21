# EPITA 2026 — Programmation par Contraintes
> Groupe de Matteo Atkinson et Paul Witkowski

## G3 - Coordination de drones par Multi-Agent Path Finding

Le Multi-Agent Path Finding (MAPF) consiste a calculer les trajectoires optimales d'un ensemble d'agents (drones, robots) partageant un espace commun, de maniere a ce qu'aucune collision ne se produise et que chaque agent atteigne son objectif. C'est un probleme combinatoire extremement difficile (NP-hard) qui se modelise naturellement en CP-SAT avec des contraintes de non-collision (pas deux agents au meme noeud au meme instant), de mouvement (deplacement vers les voisins uniquement), et d'objectif (chaque agent doit atteindre sa cible). **Note** : contrairement au Multi-robot Warehouse Task Assignment (annexe #12, EPITA 2025) qui modelise l'affectation de taches dans un entrepot sur grille 2D avec aisles predefinis, ce sujet se concentre sur la coordination de drones en espace aerien ouvert 3D avec contraintes de zones NOTAM, separation ATC (distance minimale en vol libre), conditions meteorologiques dynamiques, et obstacles tridimensionnels (batiments, lignes haute tension). L'espace de recherche est continu (pas de grille) et les contraintes de collision sont tridimensionnelles avec marges de securite.

### Objectifs
- Modeliser le MAPF comme un probleme CP-SAT avec contraintes de non-collision temporelles
- Implementer les contraintes de mouvement (grille 2D/3D), d'objectif et de non-collision (sommet et arete)
- Ajouter des contraintes de capacite (zones a trafic limite) et d'optimisation (minimiser le makespan ou le flowtime)
- Evaluer sur les benchmarks MAPF de la litterature (Moving AI Lab, grid-based)
- Comparer avec les algorithmes specialises MAPF (CBS, A* with OD, ECBS)

---

## Structure du projet

```
.
├── solver/               # Algorithmes MAPF
│   ├── grid.py           # Grille 2D/3D, voisins, obstacles
│   ├── mapf.py           # CP-SAT (MAPFSolver)
│   ├── cbs.py            # CBS + ECBS (space-time A*)
│   ├── astar.py          # A* + BFS distances
│   └── od_astar.py       # OD-A* (Operator Decomposition)
├── api/                  # Backend Flask
│   ├── server.py         # Endpoints /scenarios et /solve  →  port 5050
│   └── scenario_loader.py
├── frontend/             # Interface web 3D
│   ├── index.html
│   ├── scene.js          # Rendu 3D (Three.js)
│   ├── drones.js         # Animation drones
│   ├── ui.js             # Contrôles utilisateur
│   ├── api.js            # Appels REST
│   └── serve.py          # Serveur de dev  →  port 8080
├── notebooks/            # Analyses Jupyter
│   ├── 01_model_2d.ipynb # MAPF 2D : CP-SAT, CBS, ECBS, OD-A*
│   └── 02_model_3d.ipynb # Extension 3D avec bâtiments
├── scenarios/            # 11 scénarios JSON pré-définis
├── tests/                # Tests pytest
│   ├── test_grid.py
│   ├── test_mapf.py
│   ├── test_cbs.py
│   ├── test_od_astar.py
│   └── test_api.py
└── requirements.txt
```

---

## Lancement

### Installation

```bash
python -m venv .venv

# Windows
.venv\Scripts\activate
# macOS / Linux
source .venv/bin/activate

pip install -r requirements.txt
```

### Backend (API Flask)

```bash
python -m api.server
# → http://localhost:5050
```

Endpoints disponibles :
- `GET  /scenarios` — liste des scénarios pré-définis
- `POST /solve`     — résout une instance MAPF (méthodes : `cpsat`, `cbs`, `ecbs`, `od_astar`)

### Frontend (interface web 3D)

Dans un second terminal (le backend doit tourner en parallèle) :

```bash
cd frontend
python serve.py
# → http://localhost:8080
```

> **Important** : lancer depuis le dossier `frontend/` — `SimpleHTTPRequestHandler` sert le répertoire courant, donc `python3 -m frontend.serve` depuis la racine afficherait l'arborescence du projet au lieu de `index.html`.

Ouvrir `http://localhost:8080` dans le navigateur.

### Tests

```bash
pytest
# ou en mode verbeux
pytest -v
```

### Notebooks

```bash
jupyter lab
# ouvrir notebooks/01_model_2d.ipynb  (MAPF 2D)
# ouvrir notebooks/02_model_3d.ipynb  (extension 3D)
```

---

### References externes
- Stern, R., et al. (2019). "Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks." *Symposium on Combinatorial Search (SoCS)*. [arXiv](https://arxiv.org/abs/1906.08291)
- Sharon, G., et al. (2015). "Conflict-Based Search for Optimal Multi-Agent Pathfinding." *Artificial Intelligence*, 219, 40-66. [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S0004370214001386)
- Moving AI Lab: MAPF Benchmarks. [movingai.com](https://movingai.com/benchmarks/mapf/)
- Felner, A., et al. (2017). "Adding Heuristics to Conflict-Based Search for Multi-Agent Path Finding." *ICAPS*. [AAAI](https://ojs.aaai.org/index.php/ICAPS/article/view/13826)
