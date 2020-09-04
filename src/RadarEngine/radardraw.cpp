#include "radardraw.h"
#include "radarengine.h"

using namespace RadarEngineARND;

P2CLookupTable *lookupTable = nullptr;
P2CLookupTable* GetP2CLookupTable()
{
    if (!lookupTable)
    {
        lookupTable = (P2CLookupTable*)malloc(sizeof(P2CLookupTable));

        if (!lookupTable)
        {
            qDebug()<<Q_FUNC_INFO<<"Out Of Memory, fatal!";
            exit(1);
        }

        for (int arc = 0; arc < LINES_PER_ROTATION + 1; arc++)
        {
            GLfloat sine = cosf((GLfloat)arc * M_PI * 2 / LINES_PER_ROTATION);
            GLfloat cosine = sinf((GLfloat)arc * M_PI * 2 / LINES_PER_ROTATION);
            for (int radius = 0; radius < RETURNS_PER_LINE + 1; radius++)
            {
                lookupTable->x[arc][radius] = (GLfloat)radius * cosine/RETURNS_PER_LINE;
                lookupTable->y[arc][radius] = (GLfloat)radius * sine/RETURNS_PER_LINE;
                lookupTable->intx[arc][radius] = (int)(lookupTable->x[arc][radius]*RETURNS_PER_LINE);
                lookupTable->inty[arc][radius] = (int)(lookupTable->y[arc][radius]*RETURNS_PER_LINE);
            }
        }
    }
    return lookupTable;
}

QString RadarDraw::methods = "Vertex Array";

// Factory to generate a particular draw implementation
RadarDraw* RadarDraw::make_Draw(RadarEngine *ri, int draw_method)
{
    qDebug()<<Q_FUNC_INFO;
    switch (draw_method)
    {
    case 0:
        methods = "Vertex Array";
        return new RDVert(ri);
    case 1:
        methods = "Shader";
        return new RDVert(ri);
    default:
        qDebug()<<Q_FUNC_INFO<<"unsupported draw method "<<draw_method;
    }
    return 0;
}

RadarDraw::~RadarDraw() {}

QString RadarDraw::GetDrawingMethods() {

    return methods;
}

#define ADD_VERTEX_POINT(angle, radius, r, g, b, a)          \
{                                                          \
    line->points[count].x = m_polarLookup->x[angle][radius]; \
    line->points[count].y = m_polarLookup->y[angle][radius]; \
    line->points[count].red = r;                             \
    line->points[count].green = g;                           \
    line->points[count].blue = b;                            \
    line->points[count].alpha = a;                           \
    count++;                                                 \
}

RDVert::RDVert(RadarEngine* re):
    m_ri(re)
{
    cur_elapsed_time = Crypto::initProtect();
    if(!Crypto::checkProtect(cur_elapsed_time))
    {
        qDebug()<<"not valid";
        exit(0);
    }

    for (size_t i = 0; i < ARRAY_SIZE(m_vertices); i++)
    {
        m_vertices[i].count = 0;
        m_vertices[i].allocated = 0;
        m_vertices[i].timeout = 0;
        m_vertices[i].points = 0;
    }
    m_count = 0;
    m_oom = false;

    m_polarLookup = GetP2CLookupTable();
}

void RDVert::SetBlob(VertexLine* line, int angle_begin, int angle_end, int r1, int r2, GLubyte red, GLubyte green,
                     GLubyte blue, GLubyte alpha)
{
    //    qDebug()<<Q_FUNC_INFO<<"angle_begin "<<angle_begin<<"angle_end "<<angle_end<<"r1 "<<r1<<"r2 "<<r2
    //           <<"red "<<red<<"green "<<green<<"blue "<<blue<<"alpha "<<alpha;
    if (r2 == 0)
    {
        return;
    }
    int arc1 = MOD_ROTATION2048(angle_begin);
    int arc2 = MOD_ROTATION2048(angle_end);
    size_t count = line->count;

    if (line->count + VERTEX_PER_QUAD > line->allocated)
    {
        const size_t extra = 8 * VERTEX_PER_QUAD;
        line->points = (VertexPoint*)realloc(line->points, (line->allocated + extra) * sizeof(VertexPoint));
        line->allocated += extra;
        m_count += extra;
        qDebug()<<"extra loc";
    }

    if (!line->points)
    {
        if (!m_oom)
        {
            qDebug()<<"BR24radar_pi: Out of memory";
            m_oom = true;
        }
        return;
    }

    // First triangle
    //    qDebug()<<Q_FUNC_INFO<<arc1<<arc2<<r1<<r2;
    ADD_VERTEX_POINT(arc1, r1, red, green, blue, alpha);
    ADD_VERTEX_POINT(arc1, r2, red, green, blue, alpha);
    ADD_VERTEX_POINT(arc2, r1, red, green, blue, alpha);

    // Second triangle

    ADD_VERTEX_POINT(arc2, r1, red, green, blue, alpha);
    ADD_VERTEX_POINT(arc1, r2, red, green, blue, alpha);
    ADD_VERTEX_POINT(arc2, r2, red, green, blue, alpha);

    line->count = count;
    //    for(int i=0;i<line->count;i++)
    //        qDebug()<<"count" <<i<<"x "<<line->points[i].x<<"y "<<line->points[i].y
    //               <<"red"<<line->points[i].red<<"green"<<line->points[i].green<<"blue"<<line->points[i].blue<<"alpha"<<line->points[i].alpha;

}

void RDVert::ProcessRadarSpoke(int angle, quint8 *data, size_t len)
{
    QColor colour;
    BlobColour previous_colour = BLOB_NONE;
    GLubyte strength = 0;
    quint64 now = QDateTime::currentMSecsSinceEpoch();

    //    qDebug()<<Q_FUNC_INFO<<"transparency"<<transparency;

    int r_begin = 0;
    int r_end = 0;

    if (angle < 0 || angle >= LINES_PER_ROTATION)
    {
        return;
    }

    VertexLine* line = &m_vertices[angle];

    if (!line->points)
    {
        static size_t INITIAL_ALLOCATION = 600;
        line->allocated = INITIAL_ALLOCATION * VERTEX_PER_QUAD;
        m_count += INITIAL_ALLOCATION * VERTEX_PER_QUAD;
        line->points = (VertexPoint*)malloc(line->allocated * sizeof(VertexPoint));
        if (!line->points)
        {
            if (!m_oom)
            {
                qDebug()<<"BR24radar_pi: Out of memory";
                m_oom = true;
            }
            line->allocated = 0;
            line->count = 0;
            return;
        }
        //        qDebug()<<"init loc";

    }
    line->count = 0;
    line->timeout = now + 6000;
    //    line->timeout = now + m_ri->m_pi->m_settings.max_age;

    for (size_t radius = 0; radius < len; radius++)
    {
        strength = data[radius];

        BlobColour actual_colour = m_ri->m_colour_map[strength];
        //        qDebug()<<Q_FUNC_INFO<<"strength "<<strength<<"actual color "<<actual_colour;

        if (actual_colour == previous_colour)
        {
            // continue with same color, just register it
            r_end++;
        }
        else if (previous_colour == BLOB_NONE && actual_colour != BLOB_NONE)
        {
            // blob starts, no display, just register
            r_begin = radius;
            r_end = r_begin + 1;
            previous_colour = actual_colour;  // new color
        }
        else if (previous_colour != BLOB_NONE && (previous_colour != actual_colour))
        {
            colour = m_ri->m_colour_map_rgb[previous_colour];

            SetBlob(line, angle, angle + 1, r_begin, r_end, colour.red(), colour.green(), colour.blue(), colour.alpha());

            previous_colour = actual_colour;
            if (actual_colour != BLOB_NONE)
            {  // change of color, start new blob
                r_begin = radius;
                r_end = r_begin + 1;
            }
        }

    }

    if (previous_colour != BLOB_NONE)
    {  // Draw final blob
        colour = m_ri->m_colour_map_rgb[previous_colour];

        SetBlob(line, angle, angle + 1, r_begin, r_end, colour.red(), colour.green(), colour.blue(), colour.alpha());
    }
}

void RDVert::DrawRadarImage()
{
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    //    qDebug()<<Q_FUNC_INFO;
//    quint64 now = QDateTime::currentMSecsSinceEpoch();

    for (size_t i = 0; i < LINES_PER_ROTATION; i++)
    {
        VertexLine* line = &m_vertices[i];
        /*
        if (!line->count || TIMED_OUT(now, line->timeout))
        {
            continue;
        }
        */
        glVertexPointer(2, GL_FLOAT, sizeof(VertexPoint), &line->points[0].x);
        glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(VertexPoint), &line->points[0].red);
        glDrawArrays(GL_TRIANGLES, 0, line->count);
        //        glDrawArrays(GL_POINTS, 0, line->count);
    }

    glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    glDisableClientState(GL_COLOR_ARRAY);
}


RDVert::~RDVert()
{
    for (size_t i = 0; i < LINES_PER_ROTATION; i++)
    {
        if (m_vertices[i].points)
        {
            free(m_vertices[i].points);
        }
    }
}

GLTexture::GLTexture() : m_texture(0), m_failed(false)
{
    glGenTextures(1, &m_texture);
}

GLTexture::~GLTexture()
{
    glDeleteTextures(1, &m_texture);
}

GLTextureCube::GLTextureCube(int width, int height)
{
    glBindTexture(GL_TEXTURE_2D, m_texture);

    glTexImage2D(GL_TEXTURE_2D, 0, 4, width, height, 0,
                 GL_BGRA, GL_UNSIGNED_BYTE, 0);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void GLTextureCube::load(const QImage &img)
{
    // TODO: Add error handling.

    glBindTexture(GL_TEXTURE_2D, m_texture);

    QImage image(img);

    if (image.isNull()) m_failed = true;

    image = image.convertToFormat(QImage::Format_ARGB32);

    qDebug() << "Image size:" << image.width() << "x" << image.height();

    glTexImage2D(GL_TEXTURE_2D, 0, 4, image.width(), image.height(), 0,
                 GL_BGRA, GL_UNSIGNED_BYTE, image.bits());

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
}

void GLTextureCube::bind()
{
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glEnable(GL_TEXTURE_2D);
}

void GLTextureCube::unbind()
{
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
}
