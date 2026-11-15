# Ares Atlas — data provenance

## Bundled Jezero terrain

The desktop app bundles a cropped Jezero western delta area from the public
**Mars 2020 Science Investigation CTX DEM Mosaic** distributed by the USGS
Astrogeology Science Center and derived from NASA's Mars Reconnaissance Orbiter
Context Camera stereo imagery. The source DEM is 20 m/pixel and tied vertically
to MOLA. The bundled area is resampled to a 338 × 356 grid at approximately
20 m spacing for real-time terrain rendering and route computation.

- Dataset page: <https://astrogeology.usgs.gov/search/map/mars_2020_science_investigation_ctx_dem_mosaic>
- DEM source: <https://planetarymaps.usgs.gov/mosaic/mars2020_trn/CTX/ScienceInvestigationMaps_JPL/M20_JezeroCrater_CTXDEM_20m.tif>
- Bundled file: `web/public/assets/jezero.height.bin`
- Cropped bounds: 77.40–77.52°E, 18.38–18.50°N (planetocentric east longitude)
- Datum: Mars 2000 / MOLA areoid

A matched CTX orthomosaic from the same USGS/PDS source is bundled at 5 m/pixel
and draped over the DEM as terrain imagery:

- Orthomosaic source: <https://asc-pds-services.s3.us-west-2.amazonaws.com/mosaic/mars2020_trn/CTX/ScienceInvestigationMaps_JPL/M20_JezeroCrater_CTXortho_mosaic_5m.tif>
- Bundled crop: `web/public/assets/jezero.ctx.png`

The CTX orthomosaic is grayscale source imagery with a restrained warm display
tint; it is not natural-color photography. The mesh elevation comes from the
DEM. The app's slope-risk overlay and route cost surface are derived from the
DEM. CRISM mineralogy, THEMIS thermal inertia, MEDA conditions, and the
camera/IMU/EKF signals are not bundled as mission observations; navigation
sensor behavior is an interactive simulation for demonstrating the proposed
system.

## Attribution / use

The data is distributed by the USGS Astrogeology Science Center from the Mars
Reconnaissance Orbiter CTX archive. Credit NASA/JPL-Caltech and USGS Astrogeology.
The source product metadata and any applicable data-use terms remain authoritative.
