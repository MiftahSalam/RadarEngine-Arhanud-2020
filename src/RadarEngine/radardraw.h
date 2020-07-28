#ifndef RADARDRAW_H
#define RADARDRAW_H

#include "radarengine_global.h"
#include <Crypto/crypto.h>

#include <QtOpenGL/qgl.h>

struct P2CLookupTable {
  GLfloat x[LINES_PER_ROTATION + 1][RETURNS_PER_LINE + 1];
  GLfloat y[LINES_PER_ROTATION + 1][RETURNS_PER_LINE + 1];
  int intx[LINES_PER_ROTATION + 1][RETURNS_PER_LINE + 1];
  int inty[LINES_PER_ROTATION + 1][RETURNS_PER_LINE + 1];
};

extern P2CLookupTable *lookupTable;
P2CLookupTable* GetP2CLookupTable();

namespace RadarEngineARND {

class RadarEngine;

class RadarDraw
{
public:
    static RadarDraw* make_Draw(RadarEngine *re, int draw_method);

    virtual void DrawRadarImage() = 0;
    virtual void ProcessRadarSpoke(int angle, quint8* data, size_t len) = 0;

    virtual ~RadarDraw() = 0;

    static QString methods;
    static QString GetDrawingMethods();
};

class RDVert : public RadarDraw
{
public:
    RDVert(RadarEngine* re);
    void DrawRadarImage();
    void ProcessRadarSpoke(int angle, quint8 *data, size_t len);

    ~RDVert();
private:
    RadarEngine* m_ri;

    static const int VERTEX_PER_TRIANGLE = 3;
    static const int VERTEX_PER_QUAD = 2 * VERTEX_PER_TRIANGLE;
    static const int MAX_BLOBS_PER_LINE = RETURNS_PER_LINE;

    struct VertexPoint
    {
        GLfloat x;
        GLfloat y;
        GLubyte red;
        GLubyte green;
        GLubyte blue;
        GLubyte alpha;
    };

    struct VertexLine
    {
        VertexPoint* points;
        quint64 timeout;
        size_t count;
        size_t allocated;
    };

    P2CLookupTable* m_polarLookup;

    VertexLine m_vertices[LINES_PER_ROTATION];
    unsigned int m_count;
    bool m_oom;

    void SetBlob(VertexLine* line, int angle_begin, int angle_end, int r1, int r2, GLubyte red, GLubyte green, GLubyte blue,
                 GLubyte alpha);
};

class GLTexture
{
public:
    GLTexture();
    virtual ~GLTexture();
    virtual void bind() = 0;
    virtual void unbind() = 0;
    virtual bool failed() const {return m_failed;}
protected:
    GLuint m_texture;
    bool m_failed;
};

class GLTextureCube : public GLTexture
{
public:
    explicit GLTextureCube(int width, int height);
    void load(const QImage &img);
    void bind();
    void unbind();
};

}
#endif // RADARDRAW_H
