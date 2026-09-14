package org.bimanual.questcapture;

import android.app.NativeActivity;
import android.os.Bundle;
import java.util.Locale;
import java.util.UUID;

public final class MainActivity extends NativeActivity {
    private String sessionId;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        String requested = getIntent().getStringExtra("session_id");
        if (requested != null && !requested.matches("[0-9a-fA-F]{32}")) {
            throw new IllegalArgumentException("session_id must be a 32-character UUID hex string");
        }
        sessionId = requested == null
                ? UUID.randomUUID().toString().replace("-", "")
                : requested.toLowerCase(Locale.ROOT);
        // NativeActivity starts the native thread inside onCreate.
        super.onCreate(savedInstanceState);
    }

    public String getSessionId() {
        return sessionId;
    }
}

