# three.js 0.160.0 (só o que a vista 3D usa)

Copiado do pacote npm `three@0.160.0` (licença MIT, em `LICENSE`), baixado com `npm pack` em
01/11/2026. O sha512 do pacote confere com o `dist.integrity` do registro do npm:
`sha512-DLU8lc0zNIPkM7rH5/e1Ks1Z8tWCGRq6g8mPowdDJpw1CFBJMU7UoJjC6PefXW7z//SSl0b2+GCw14LB+uDhng==`.

| Arquivo aqui | No pacote |
| --- | --- |
| `three.module.min.js` | `build/three.module.min.js`, sem mudança |
| `addons/controls/OrbitControls.js` | `examples/jsm/controls/OrbitControls.js` |
| `addons/renderers/CSS2DRenderer.js` | `examples/jsm/renderers/CSS2DRenderer.js` |
| `LICENSE` | `LICENSE`, sem mudança |

A única mudança está nos dois arquivos de `addons/`: a linha `} from 'three';` virou
`} from '../../three.module.min.js';`. Assim os módulos se acham pelo caminho relativo, e a página não
precisa de import map.

sha256 dos arquivos como estão aqui:

```text
3e690ac7d180b0aadf0891bea39eec643e29e2d3e75c99b18689518665f69ba6  three.module.min.js
e732d778b0a9fcc7833d7f5b7c7344fb1e2e98b4b4ed6cd67993d9153966215e  addons/controls/OrbitControls.js
c4de86546a39e6f695824668c49ef961eb516bf5903117480ef4b52f2c2749b0  addons/renderers/CSS2DRenderer.js
852e0e8699169bf9f6fdc6bda3e682d078dcbc738b5d33e74df594721bff271d  LICENSE
```
