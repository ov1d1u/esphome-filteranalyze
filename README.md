# ESP32 camera fill measurement

`esp32cam.yaml` now uses the local `fill_level` ESPHome external component. It
analyzes camera frames on the ESP32 and publishes **Filter Fill Percentage** and
**Filter Match Confidence** to Home Assistant. The Raspberry Pi endpoint and
Home Assistant image upload are no longer needed for this measurement.

The component measures once when the device starts and every 12 hours after the
most recent measurement. **Measure Filter Now** starts the same sequence on
demand. It turns on the flashlight, discards older camera frames while exposure
settles for at least three seconds, requests a fresh frame, then turns off the
flashlight it turned on. A camera timeout also turns it off and makes the
sensors unavailable. The flashlight retains its 60-second safety timeout for
manual use. Ordinary Home Assistant camera requests do not update the fill
sensors.

The two images in `templates/` are embedded as grayscale data when ESPHome
compiles the firmware. They must be crops from the same 640×480 camera view and
orientation used at runtime. The component first locates the best template with
normalized correlation, then applies the same 5-pixel-wide ROI offset, HSV
threshold, 5×5 morphology, 150-pixel blob filter, and bottom-up row count as the
Pi script. A missing or weak match makes the fill sensor unavailable (`NaN`),
rather than reporting a misleading zero.

After a valid measurement, the fill sensor accepts a value when it has no
available reading yet, when the measured level decreases, or when it increases
by more than five percentage points. Smaller increases leave the current value
unchanged. A valid zero is accepted as a decrease.

To build, provide your existing `secrets.yaml` with `api_encryption_key` next to
`esp32cam.yaml`, then run `esphome compile esp32cam.yaml`. ESP32 PSRAM is required
for JPEG decoding. The component accepts JPEG frames up to 640×480. Match
confidence must reach 0.4 by default; `min_confidence`, `min_blob_area`,
`interval`, and `stabilization_delay` can be adjusted under `fill_level:`.

The fine match uses every template pixel. Its first two passes sample pixels to
reduce work, so the selected location can differ from a full OpenCV search on
unusual images. Check the confidence sensor and fill percentage against a few
known images before relying on the reading in an automation.
