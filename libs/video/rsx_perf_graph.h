/* Uses the existing present-interval ring; no sampling thread or file output. */
static void racit_perf_overlay(ui_out* out, const rsx_ui_font* font)
{
    double frames[128]; unsigned count;
    char summary[64];
    LOCK();
    memcpy(summary, s_fps_text, sizeof summary);
    count = s_ft_n < N_OF(s_ft) ? s_ft_n : N_OF(s_ft);
    for (unsigned i = 0; i < count; ++i)
        frames[i] = s_ft[(s_ft_n - count + i) % N_OF(s_ft)];
    UNLOCK();
    if (!summary[0]) snprintf(summary, sizeof summary, "-- FPS / -- ms");
    const float px = (float)font->px, x = px * .6f, y = px * .5f;
    const float width = px * 21, height = px * 2.4f, line = (float)font->line_h;
    rect(out, 0, 0, x + width + px, y + line * 3 + height * 2 + px,
         rgba(0, 0, 0, 180));
    text(out, x, y, summary, rgba(255, 230, 120, 255));
    for (int graph = 0; graph < 2; ++graph) {
        const float top = y + line * (graph ? 3 : 2) + height * graph;
        double maximum = graph ? 33.333 : 120.0;
        for (unsigned i = 0; i < count; ++i) {
            double value = graph ? frames[i] : frames[i] > 0 ? 1000.0 / frames[i] : 0;
            if (value > maximum) maximum = value;
        }
        char label[64];
        snprintf(label, sizeof label, graph ? "Frametime: 0 - %.1f ms" : "FPS: 0 - %.0f", maximum);
        text(out, x, top - line, label, rgba(210, 220, 235, 255));
        rect(out, x, top, x + width, top + height, rgba(35, 42, 52, 210));
        const float guide = top + height * (1.f - (float)((graph ? 16.666667 : 60.0) / maximum));
        rect(out, x, guide, x + width, guide + 1, rgba(125, 130, 140, 180));
        for (unsigned i = 0; i < count; ++i) {
            const double value = graph ? frames[i] : frames[i] > 0 ? 1000.0 / frames[i] : 0;
            const float left = x + width * (float)(128 - count + i) / 128.f;
            rect(out, left, top + height * (1.f - (float)(value / maximum)),
                 left + width / 128.f, top + height,
                 graph ? rgba(100, 190, 255, 240) : rgba(120, 230, 160, 240));
        }
    }
}
