# Filtrage numérique — vue d'ensemble

## Principe général
Un filtre numérique transforme une suite d'échantillons en une nouvelle suite dont le contenu spectral correspond mieux à une spécification donnée. Il détecte les composantes indésirables (bruit, bandes hors intérêt) et les atténue tout en préservant la bande utile. Le système étant discret, toutes les opérations se font à fréquence d'échantillonnage fixe $F_s$ au moyen d'équations aux différences.

## Grandes familles de filtres
- **FIR (Finite Impulse Response)** : la sortie est une somme pondérée d'un nombre fini d'échantillons passés. Avantages : stabilité garantie et phase linéaire; inconvénient : transitions abruptes coûteuses en nombre de taps.
- **IIR (Infinite Impulse Response)** : la sortie réutilise des sorties passées en plus des entrées courantes. Les transitions sont plus raides pour un petit nombre de coefficients, mais la phase est non linéaire et la stabilité dépend du placement des pôles.
- **Autres familles** : filtres adaptatifs (LMS, RLS), filtres de Kalman, filtres multirate/décimateurs, etc., chacun répondant à un objectif précis (estimation optimale, réduction adaptative de bruit, changement d'échelle temporelle, etc.).

## Zoom sur les filtres IIR
Un IIR suit une équation du type :
$$y[n] = \sum_{k=0}^{M} b_k x[n-k] - \sum_{k=1}^{N} a_k y[n-k]$$
Cette forme rappelle la topologie des filtres analogiques (Butterworth, Chebyshev, Bessel). Elle permet d'obtenir des pentes marquées avec peu de coefficients. En contrepartie, la phase est non linéaire, les coefficients doivent être soigneusement quantifiés, et il faut garantir que les pôles restent à l'intérieur du cercle unité pour maintenir la stabilité.

### Filtre IIR matériel sur ESP32-C6
Le driver ADC continu de l'ESP32-C6 expose un filtre IIR matériel qui se comporte comme une moyenne exponentielle (EMA) :
$$y[n] = y[n-1] + \frac{x[n] - y[n-1]}{K}$$
avec $K \in \{2,4,8,16,32,64\}$. Un $K$ faible laisse passer plus de bande (lissage léger), un $K$ élevé lisse fortement et augmente la latence. La fréquence de coupure dépend de la fréquence d'échantillonnage :
$$f_c \approx \frac{F_s}{2\pi K}$$
Ainsi, doubler $F_s$ double la bande passante; à $F_s$ constant, augmenter $K$ resserre la bande.

### Impact sur la mesure d'un sinus 50 Hz
Pour un signal sinusoïdal 50 Hz, activer ce filtre réduit le bruit haute fréquence avant le calcul des statistiques (min/max/moyenne) ou d'une valeur RMS. Le résultat est une amplitude plus stable mais avec un léger retard (~$K/2$ échantillons) et une réponse plus douce aux transitoires. Lors d'un calcul RMS, l'IIR est généralement bénéfique car il atténue les pointes aléatoires avant l'opération de mise au carré. Il faut toutefois que la fréquence de coupure reste au-dessus de la fondamentale : avec $F_s = 2\,\text{kHz}$ et $K = 8$, on obtient seulement ~40 Hz, ce qui atténue la sinusoïde. Monter $F_s$ vers $2\pi K f_c \approx 2{,}5\,\text{kHz}$ permet de garder $f_c \approx 50$ Hz tout en conservant le lissage exponentiel.
